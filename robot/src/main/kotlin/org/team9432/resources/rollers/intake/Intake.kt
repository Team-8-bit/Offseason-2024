package org.team9432.resources.rollers.intake

import org.littletonrobotics.junction.Logger

class Intake(private val io: IntakeIO) {
    private val inputs = LoggedIntakeIOInputs()

    var goal = Goal.IDLE

    enum class Goal(val voltage: Double) {
        FLOOR_INTAKE(10.0),
        LOAD(2.0),
        FLOOR_EJECT(-5.0),
        IDLE(0.0)
    }

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Rollers/Intake", inputs)
        Logger.recordOutput("Rollers/IntakeState", goal)
        io.setVoltage(goal.voltage)
    }
}