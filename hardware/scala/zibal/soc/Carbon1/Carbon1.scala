package zibal.soc

import spinal.core._
import spinal.core.sim._
import spinal.lib._

import zibal.platform.Carbon
import zibal.board.BoardParameter

import zibal.misc.{ElementsConfig, EmbenchIotTools}

import spinal.lib.bus.amba3.apb._
import spinal.lib.bus.amba4.axi._

import zibal.peripherals.io.gpio.{Apb3Gpio, Gpio, GpioCtrl}
import zibal.peripherals.com.uart.{Apb3Uart, Uart, UartCtrl}
import zibal.peripherals.com.spi.{Apb3SpiMaster, Spi, SpiCtrl}
import zibal.peripherals.com.i2c.{Apb3I2cController, I2c, I2cCtrl}


object Carbon1 {
  def apply(parameter: Carbon.Parameter) = Carbon1(parameter)

  def prepare(soc: Carbon1, elementsConfig: ElementsConfig.ElementsConfig) {
    val cpuClockDomain = soc.clockCtrl.getClockDomainByName("system")
    EmbenchIotTools(elementsConfig).generate(cpuClockDomain.frequency.getValue)
  }

  case class Parameter(boardParameter: BoardParameter) extends SocParameter(boardParameter, 4) {
    val uartStd = UartCtrl.Parameter.default
    val gpioStatus = GpioCtrl.Parameter(4, 2, (0 to 2), (3 to 3), (3 to 3))
    val gpioA = GpioCtrl.Parameter(8, 2, null, null, null)
    val i2cA = I2cCtrl.Parameter.default
  }

  case class Carbon1(parameter: Carbon.Parameter) extends Carbon.Carbon(parameter) {
    var socParameter = parameter.getSocParameter.asInstanceOf[Parameter]
    val io_per = new Bundle {
      val uartStd = master(Uart.Io(socParameter.uartStd))
      val gpioStatus = Gpio.Io(socParameter.gpioStatus)
      val gpioA = Gpio.Io(socParameter.gpioA)
      val i2cA = master(I2c.Io(socParameter.i2cA))
    }

    val peripherals = new ClockingArea(clockCtrl.getClockDomainByName("system")) {

      val uartStdCtrl = Apb3Uart(socParameter.uartStd)
      uartStdCtrl.io.uart <> io_per.uartStd
      addApbDevice(uartStdCtrl.io.bus, 0x00000, 4 kB)
      addInterrupt(uartStdCtrl.io.interrupt)

      val gpioStatusCtrl = Apb3Gpio(socParameter.gpioStatus)
      gpioStatusCtrl.io.gpio <> io_per.gpioStatus
      addApbDevice(gpioStatusCtrl.io.bus, 0x10000, 4 kB)
      addInterrupt(gpioStatusCtrl.io.interrupt)

      val gpioACtrl = Apb3Gpio(socParameter.gpioA)
      gpioACtrl.io.gpio <> io_per.gpioA
      addApbDevice(gpioACtrl.io.bus, 0x11000, 4 kB)
      addInterrupt(gpioACtrl.io.interrupt)

      val i2cControllerACtrl = Apb3I2cController(socParameter.i2cA)
      i2cControllerACtrl.io.i2c <> io_per.i2cA
      addApbDevice(i2cControllerACtrl.io.bus, 0x50000, 4 kB)
      addInterrupt(i2cControllerACtrl.io.interrupt)

      connectPeripherals()
    }
  }
}
