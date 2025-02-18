package zibal.soc.helium2

import spinal.core._
import spinal.core.sim._
import spinal.lib._

import zibal.peripherals.system.reset.ResetControllerCtrl.ResetControllerCtrl
import zibal.peripherals.system.clock.ClockControllerCtrl.ClockControllerCtrl

import zibal.board.{KitParameter, BoardParameter, ResetParameter, ClockParameter}
import zibal.board.DH008
import zibal.platform.Helium
import zibal.soc.Helium2
import zibal.misc.{ElementsConfig, BinTools, XilinxTools, SimulationHelper, TestCases}
import zibal.blackboxes.xilinx.a7._


object DH008Board {
  def apply(source: String) = DH008Board(source)

  def main(args: Array[String]) {
    val elementsConfig = ElementsConfig(this)

    val compiled = elementsConfig.genFPGASimConfig.compile {
      val board = DH008Board(args(0))
      board.top.soc.initOnChipRam(elementsConfig.zephyrBuildPath + "/zephyr.bin")
      for (domain <- board.top.soc.parameter.getKitParameter.clocks) {
        board.top.soc.clockCtrl.getClockDomainByName(domain.name).clock.simPublic()
      }
      board
    }
    args(1) match {
      case "simulate" =>
        compiled.doSimUntilVoid("simulate") { dut =>
          dut.simHook()
          val testCases = TestCases()
          testCases.addClock(dut.io.clock, DH008.oscillatorFrequency, 100 ms)
          testCases.dump(dut.io.uartStd.txd, dut.baudPeriod)
        }
      case "boot" =>
        compiled.doSimUntilVoid("boot") { dut =>
          dut.simHook()
          val testCases = TestCases()
          testCases.addClockWithTimeout(dut.io.clock, DH008.oscillatorFrequency, 10 ms)
          testCases.boot(dut.io.uartStd.txd, dut.baudPeriod)
        }
      case "mtimer" =>
        compiled.doSimUntilVoid("mtimer") { dut =>
          dut.simHook()
          val testCases = TestCases()
          testCases.addClockWithTimeout(dut.io.clock, DH008.oscillatorFrequency, 400 ms)
          testCases.heartbeat(dut.io.gpioStatus(0))
        }
      case _ =>
        println(s"Unknown simulation ${args(1)}")
    }
  }

  case class DH008Board(source: String) extends Component {
    val io = new Bundle {
      val clock = inout(Analog(Bool))
      val jtag = new Bundle {
        val tdo = inout(Analog(Bool))
      }
      val uartStd = new Bundle {
        val txd = inout(Analog(Bool))
        val rxd = inout(Analog(Bool))
        val rts = inout(Analog(Bool))
        val cts = inout(Analog(Bool))
      }
      val gpioStatus = Vec(inout(Analog(Bool())), 4)
      val gpioA = Vec(inout(Analog(Bool())), 12)
      val vgaA = new Bundle {
        val hSync = inout(Analog(Bool))
        val vSync = inout(Analog(Bool))
        val enable = inout(Analog(Bool))
        val red = Vec(inout(Analog(Bool())), 4)
        val green = Vec(inout(Analog(Bool())), 4)
        val blue = Vec(inout(Analog(Bool())), 4)
      }
    }

    val top = DH008Top()
    val analogFalse = Analog(Bool)
    analogFalse := False
    val analogTrue = Analog(Bool)
    analogTrue := True

    top.io.clock.PAD := io.clock

    top.io.jtag.tms.PAD := analogFalse
    top.io.jtag.tdi.PAD := analogFalse
    io.jtag.tdo := top.io.jtag.tdo.PAD
    top.io.jtag.tck.PAD := analogFalse
    top.io.uartStd.rxd.PAD := io.uartStd.rxd
    io.uartStd.txd := top.io.uartStd.txd.PAD
    top.io.uartStd.cts.PAD := io.uartStd.cts
    io.uartStd.rts := top.io.uartStd.rts.PAD

    for (index <- 0 until 4) {
      io.gpioStatus(index) <> top.io.gpioStatus(index).PAD
    }

    for (index <- 0 until 12) {
      io.gpioA(index) <> top.io.gpioA(index).PAD
    }

    io.vgaA.hSync := top.io.vgaA.hSync.PAD
    io.vgaA.vSync := top.io.vgaA.vSync.PAD
    io.vgaA.enable := top.io.vgaA.enable.PAD
    for (index <- 0 until 4) {
      io.vgaA.red(index) := top.io.vgaA.red(index).PAD
      io.vgaA.green(index) := top.io.vgaA.green(index).PAD
      io.vgaA.blue(index) := top.io.vgaA.blue(index).PAD
    }

    val baudPeriod = top.soc.socParameter.uartStd.init.getBaudPeriod()

    def simHook() {
      for ((domain, index) <- top.soc.parameter.getKitParameter.clocks.zipWithIndex) {
        val clockDomain = top.soc.clockCtrl.getClockDomainByName(domain.name)
        SimulationHelper.generateEndlessClock(clockDomain.clock, domain.frequency)
      }
    }
  }
}


object DH008Top {
  def apply() = DH008Top(getConfig)

  def getConfig = {
    val resets = List[ResetParameter](ResetParameter("system", 64), ResetParameter("debug", 64))
    val clocks = List[ClockParameter](
      ClockParameter("system", 100 MHz, "system"),
      ClockParameter("debug", 100 MHz, "debug", synchronousWith = "system")
    )

    val kitParameter = KitParameter(resets, clocks)
    val boardParameter = DH008.Parameter(kitParameter)
    val socParameter = Helium2.Parameter(boardParameter)
    Helium.Parameter(socParameter, 128 kB,
      (resetCtrl: ResetControllerCtrl, _, clock: Bool) => { resetCtrl.buildXilinx(clock) },
      (clockCtrl: ClockControllerCtrl, resetCtrl: ResetControllerCtrl, clock: Bool) => {
        clockCtrl.buildXilinxPll(clock, boardParameter.getOscillatorFrequency,
          List("system", "debug"), 10)
      })
  }


  def main(args: Array[String]) {
    val elementsConfig = ElementsConfig(this)
    val spinalConfig = elementsConfig.genFPGASpinalConfig

    spinalConfig.generateVerilog({
      args(0) match {
        case "prepare" =>
          val soc = Helium2(getConfig)
          Helium2.prepare(soc, elementsConfig)
          soc
        case _ =>
          val top = DH008Top(getConfig)
          top.soc.initOnChipRam(elementsConfig.zephyrBuildPath + "/zephyr.bin")
          val xdc = XilinxTools.Xdc(elementsConfig)
          top.soc.clockCtrl.generatedClocks foreach { clock => xdc.addGeneratedClock(clock) }
          xdc.generate(top.io)
          top
      }
    })
  }

  case class DH008Top(parameter: Helium.Parameter) extends Component {
    var boardParameter = parameter.getBoardParameter.asInstanceOf[DH008.Parameter]
    val io = new Bundle {
      val clock = XilinxCmosIo("E12").clock(boardParameter.getOscillatorFrequency)
      val jtag = new Bundle {
        val tms = XilinxCmosIo("R13")
        val tdi = XilinxCmosIo("N13")
        val tdo = XilinxCmosIo("P13")
        val tck = XilinxCmosIo("N14").clock(boardParameter.getJtagFrequency)
      }
      val uartStd = new Bundle {
        val txd = XilinxCmosIo("M4")
        val rxd = XilinxCmosIo("L4")
        val rts = XilinxCmosIo("M2")
        val cts = XilinxCmosIo("M1")
      }
      val gpioStatus = Vec(XilinxCmosIo("K12"), XilinxCmosIo("L13"), XilinxCmosIo("K13"),
                           XilinxCmosIo("G11"))
      val gpioA = Vec(XilinxCmosIo("F2"), XilinxCmosIo("E1"), XilinxCmosIo("G5"),
                      XilinxCmosIo("G4"), XilinxCmosIo("G1"), XilinxCmosIo("G2"),
                      XilinxCmosIo("T15"), XilinxCmosIo("R15"), XilinxCmosIo("L5"),
                      XilinxCmosIo("T14"), XilinxCmosIo("R10"), XilinxCmosIo("R11"))
      val vgaA = new Bundle {
        val hSync = XilinxCmosIo("T9")
        val vSync = XilinxCmosIo("T10")
        val enable = XilinxCmosIo("N16")
        val red = Vec(XilinxCmosIo("K3"), XilinxCmosIo("K2"), XilinxCmosIo("R5"),
                      XilinxCmosIo("T5"))
        val green = Vec(XilinxCmosIo("P4"), XilinxCmosIo("P3"), XilinxCmosIo("P10"),
                        XilinxCmosIo("P11"))
        val blue = Vec(XilinxCmosIo("R6"), XilinxCmosIo("R7"), XilinxCmosIo("H11"),
                       XilinxCmosIo("G12"))
      }
    }

    val soc = Helium2(parameter)

    io.clock <> IBUF(soc.io_plat.clock)

    io.jtag.tms <> IBUF(soc.io_plat.jtag.tms)
    io.jtag.tdi <> IBUF(soc.io_plat.jtag.tdi)
    io.jtag.tdo <> OBUF(soc.io_plat.jtag.tdo)
    io.jtag.tck <> IBUF(soc.io_plat.jtag.tck)

    io.uartStd.txd <> OBUF(soc.io_per.uartStd.txd)
    io.uartStd.rxd <> IBUF(soc.io_per.uartStd.rxd)
    io.uartStd.rts <> OBUF(soc.io_per.uartStd.rts)
    io.uartStd.cts <> IBUF(soc.io_per.uartStd.cts)

    for (index <- 0 until 4) {
      io.gpioStatus(index) <> IOBUF(soc.io_per.gpioStatus.pins(index))
    }

    for (index <- 0 until 12) {
      io.gpioA(index) <> IOBUF(soc.io_per.gpioA.pins(index))
    }

    val vgaEnable = True
    io.vgaA.hSync <> OBUF(soc.io_per.vgaA.hSync)
    io.vgaA.vSync <> OBUF(soc.io_per.vgaA.vSync)
    io.vgaA.enable <> OBUF(vgaEnable)
    for (index <- 0 until 4) {
      io.vgaA.red(index) <> OBUF(soc.io_per.vgaA.pixels.r(index))
      io.vgaA.green(index) <> OBUF(soc.io_per.vgaA.pixels.g(index))
      io.vgaA.blue(index) <> OBUF(soc.io_per.vgaA.pixels.b(index))
    }
  }
}
