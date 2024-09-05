package org.team9432.resources.intake

import org.littletonrobotics.junction.Logger
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.resource.Resource
import org.team9432.lib.util.simSwitch

object Intake: Resource("Intake") {
    private val io = simSwitch(real = IntakeIONeo, sim = IntakeIOSim)
    private val inputs = LoggedIntakeIOInputs()

    private var state = State.IDLE

    enum class State(val getVoltage: () -> Double) {
        INTAKE({ 10.0 }),
        LOAD({ 2.0 }),
        OUTTAKE({ -5.0 }),
        IDLE({ 0.0 });
    }

    init {
        RobotPeriodicManager.startPeriodic { trackState(); akitUpdate() }
    }

    private fun trackState() {
        io.setVoltage(state.getVoltage())
    }

    fun setState(state: State) {
        Intake.state = state
        trackState()
        Logger.recordOutput("Intake/State", Intake.state)
    }

    val isIntaking get() = state == State.INTAKE
    val isIdle get() = state == State.IDLE

    override fun akitUpdate() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }
}