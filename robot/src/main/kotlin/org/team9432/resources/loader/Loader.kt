package org.team9432.resources.loader


import org.littletonrobotics.junction.Logger
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.resource.Resource

object Loader: Resource("Loader") {
    private val io = LoaderIONeo
    private val inputs = LoggedLoaderIOInputs()

    private var state = State.IDLE

    enum class State(val getVoltage: () -> Double) {
        INTAKE({ 8.0 }),
        LOAD({ 2.0 }),
        REVERSE({ -8.0 }),
        IDLE({ 0.0 });
    }

    init {
        RobotPeriodicManager.startPeriodic { trackState() }
    }

    override fun akitUpdate() {
        io.updateInputs(inputs)
        Logger.processInputs("Loader", inputs)
    }

    private fun trackState() {
        io.setVoltage(state.getVoltage())
    }

    fun setState(state: State) {
        Loader.state = state
        trackState()
        Logger.recordOutput("Loader/State", state)
    }

    val isIdle get() = state == State.IDLE
}