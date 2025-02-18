package zibal.board

import spinal.core._
import spinal.lib._


object AX7035 {

  val oscillatorFrequency = 50 MHz

  case class Parameter(
    kitParameter: KitParameter
  ) extends BoardParameter(
    kitParameter,
    oscillatorFrequency
  ) {
    def getJtagFrequency = 10 MHz
  }
}
