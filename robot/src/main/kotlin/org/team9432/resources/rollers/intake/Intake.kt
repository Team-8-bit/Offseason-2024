package org.team9432.resources.rollers.intake

import org.littletonrobotics.junction.Logger
import org.team9432.lib.dashboard.LoggedTunableNumber

class Intake(private val io: IntakeIO) {
    private val noteDetectionAmps by LoggedTunableNumber("Rollers/NoteDetectionAmps", 20.0)
    private val inputs = LoggedIntakeIOInputs()

    var goal = Goal.IDLE

    enum class Goal(val voltage: Double) {
        FLOOR_INTAKE(10.0),
        LOAD(2.0),
        FLOOR_EJECT(-5.0),
        IDLE(0.0)
    }

    val noteCurrentExceeded get() = (inputs.leaderSupplyCurrentAmps + inputs.followerSupplyCurrentAmps) / 2 > noteDetectionAmps

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Rollers/Intake", inputs)
        Logger.recordOutput("Rollers/IntakeState", goal)
        Logger.recordOutput("Rollers/NoteCurrentExceeded", noteCurrentExceeded)
        io.setVoltage(goal.voltage)
    }
}