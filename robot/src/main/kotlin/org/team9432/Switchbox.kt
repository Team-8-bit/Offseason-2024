package org.team9432

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID

class Switchbox(port: Int): CommandGenericHID(port) {
    val switchOne get() = super.button(1)
    val switchTwo get() = super.button(2)
    val switchThree get() = super.button(3)
    val switchFour get() = super.button(4)
    val switchFive get() = super.button(5)
    val switchSix get() = super.button(6)
    val switchSeven get() = super.button(7)
    val switchEight get() = super.button(8)
}