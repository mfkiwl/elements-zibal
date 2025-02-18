package zibal.soc.hydrogen1

import spinal.core._
import spinal.core.sim._
import spinal.lib._

import zibal.peripherals.system.reset.ResetControllerCtrl.ResetControllerCtrl
import zibal.peripherals.system.clock.ClockControllerCtrl.ClockControllerCtrl

import zibal.board.{KitParameter, BoardParameter, ResetParameter, ClockParameter}
import zibal.board.AX7101
import zibal.platform.Hydrogen
import zibal.soc.Hydrogen1
import zibal.misc.{ElementsConfig, BinTools, XilinxTools, SimulationHelper, TestCases}
import zibal.blackboxes.xilinx.a7._


object AX7101Board {
  def apply(source: String) = AX7101Board(source)

  def main(args: Array[String]) {
    val elementsConfig = ElementsConfig(this)

    val compiled = elementsConfig.genFPGASimConfig.compile {
      val board = AX7101Board(args(0))
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
          testCases.addClock(dut.io.clock, AX7101.oscillatorFrequency, 10 ms)
          testCases.dump(dut.io.uartStd.txd, dut.baudPeriod)
        }
      case "boot" =>
        compiled.doSimUntilVoid("boot") { dut =>
          dut.simHook()
          val testCases = TestCases()
          testCases.addClockWithTimeout(dut.io.clock, AX7101.oscillatorFrequency, 10 ms)
          testCases.boot(dut.io.uartStd.txd, dut.baudPeriod)
        }
      case "mtimer" =>
        compiled.doSimUntilVoid("mtimer") { dut =>
          dut.simHook()
          val testCases = TestCases()
          testCases.addClockWithTimeout(dut.io.clock, AX7101.oscillatorFrequency, 400 ms)
          testCases.heartbeat(dut.io.gpioStatus(0))
        }
      case _ =>
        println(s"Unknown simulation ${args(1)}")
    }
  }

  case class AX7101Board(source: String) extends Component {
    val io = new Bundle {
      val clock = in(Bool)
      val uartStd = new Bundle {
        val txd = inout(Analog(Bool))
        val rxd = inout(Analog(Bool))
      }
      val gpioStatus = Vec(inout(Analog(Bool())), 4)
    }

    val top = AX7101Top()
    val analogFalse = Analog(Bool)
    analogFalse := False
    val analogTrue = Analog(Bool)
    analogTrue := True

    top.io.clockPos.PAD := io.clock
    top.io.clockNeg.PAD := False

    top.io.uartStd.rxd.PAD := io.uartStd.rxd
    io.uartStd.txd := top.io.uartStd.txd.PAD

    for (index <- 0 until 4) {
      io.gpioStatus(index) <> top.io.gpioStatus(index).PAD
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


object AX7101Top {
  def apply() = AX7101Top(getConfig)

  def getConfig = {
    val resets = List[ResetParameter](ResetParameter("system", 64), ResetParameter("debug", 64))
    val clocks = List[ClockParameter](
      ClockParameter("system", 105 MHz, "system"),
      ClockParameter("debug", 105 MHz, "debug", synchronousWith = "system")
    )

    val kitParameter = KitParameter(resets, clocks)
    val boardParameter = AX7101.Parameter(kitParameter)
    val socParameter = Hydrogen1.Parameter(boardParameter)
    Hydrogen.Parameter(socParameter, 128 kB,
      (resetCtrl: ResetControllerCtrl, _, clock: Bool) => { resetCtrl.buildXilinx(clock) },
      (clockCtrl: ClockControllerCtrl, resetCtrl: ResetControllerCtrl, clock: Bool) => {
        clockCtrl.buildXilinxPll(clock, boardParameter.getOscillatorFrequency,
          List("system", "debug"), 5)
      })
  }

  def main(args: Array[String]) {
    val elementsConfig = ElementsConfig(this)
    val spinalConfig = elementsConfig.genFPGASpinalConfig

    spinalConfig.generateVerilog({
      args(0) match {
        case "prepare" =>
          val soc = Hydrogen1(getConfig)
          Hydrogen1.prepare(soc, elementsConfig)
          soc
        case _ =>
          val top = AX7101Top(getConfig)
          top.soc.initOnChipRam(elementsConfig.zephyrBuildPath + "/zephyr.bin")
          val xdc = XilinxTools.Xdc(elementsConfig)
          top.soc.clockCtrl.generatedClocks foreach { clock => xdc.addGeneratedClock(clock) }
          xdc.generate(top.io)
          top
      }
    })
  }

  case class AX7101Top(parameter: Hydrogen.Parameter) extends Component {
    var boardParameter = parameter.getBoardParameter.asInstanceOf[AX7101.Parameter]
    val io = new Bundle {
      val clockPos = XilinxLvdsInput.Pos("R4").clock(boardParameter.getOscillatorFrequency).ioStandard("DIFF_SSTL15")
      val clockNeg = XilinxLvdsInput.Neg("T4").ioStandard("DIFF_SSTL15")
      val uartStd = new Bundle {
        val txd = XilinxCmosIo("AB15")
        val rxd = XilinxCmosIo("AA15")
      }
      val gpioStatus = Vec(
        XilinxCmosIo("W5").ioStandard("LVCMOS15"),
        XilinxCmosIo("E17"),
        XilinxCmosIo("F16"),
        XilinxCmosIo("T6").ioStandard("LVCMOS15")
      )
    }

    val soc = Hydrogen1(parameter)

    val clockBuf = IBUFDS(soc.io_plat.clock)
    io.clockPos <> clockBuf
    io.clockNeg <> clockBuf

    soc.io_plat.jtag.tms := False
    soc.io_plat.jtag.tdi := False
    soc.io_plat.jtag.tck := False

    io.uartStd.txd <> OBUF(soc.io_per.uartStd.txd)
    io.uartStd.rxd <> IBUF(soc.io_per.uartStd.rxd)
    soc.io_per.uartStd.cts := False

    for (index <- 0 until 4) {
      io.gpioStatus(index) <> IOBUF(soc.io_per.gpioStatus.pins(index))
    }
  }
}
