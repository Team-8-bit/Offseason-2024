package org.team9432.resources.rollers.loader


import org.littletonrobotics.junction.Logger

class Loader(private val io: LoaderIO) {
    private val inputs = LoggedLoaderIOInputs()

    var goal = Goal.IDLE

    enum class Goal(val voltage: Double) {
        FLOOR_INTAKE(8.0),
        SHOOTER_FEED(2.0),
        ALIGN_FORWARD(2.0),
        ALIGN_REVERSE(-8.0),
        REVERSE(-8.0),
        IDLE(0.0);
    }

    val isIdle get() = goal == Goal.IDLE

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Rollers/Loader", inputs)
        Logger.recordOutput("Rollers/LoaderState", goal)
        io.setVoltage(goal.voltage)
    }
}