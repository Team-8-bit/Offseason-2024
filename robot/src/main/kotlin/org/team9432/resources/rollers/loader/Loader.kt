package org.team9432.resources.rollers.loader


import org.littletonrobotics.junction.Logger
import org.team9432.lib.dashboard.LoggedTunableNumber

class Loader(private val io: LoaderIO) {
    private val inputs = LoggedLoaderIOInputs()

    var goal = Goal.IDLE

    enum class Goal(val getVoltage: () -> Double) {
        FLOOR_INTAKE(LoggedTunableNumber("Loader/FloorIntakeVoltage", 8.0)),
        SHOOTER_FEED(LoggedTunableNumber("Loader/ShooterFeedVoltage", 2.0)),
        ALIGN_FORWARD(LoggedTunableNumber("Loader/AlignForwardVoltage", 5.0)),
        ALIGN_REVERSE(LoggedTunableNumber("Loader/AlignReverseVoltage", -3.0)),
        REVERSE(LoggedTunableNumber("Loader/ReverseVoltage", -8.0)),
        IDLE(LoggedTunableNumber("Loader/IdleVoltage", 0.0));
    }

    val isIdle get() = goal == Goal.IDLE

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Rollers/Loader", inputs)
        Logger.recordOutput("Rollers/LoaderState", goal)
        io.setVoltage(goal.getVoltage())
    }
}