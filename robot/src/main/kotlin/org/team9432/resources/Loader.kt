package org.team9432.resources

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.doglog.Logger
import org.team9432.lib.resource.Resource

object Loader: Resource("Loader") {
    private val motor = CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless)

    private var state = State.IDLE

    enum class State(val getVoltage: () -> Double) {
        INTAKE({ 8.0 }),
        LOAD({ 2.0 }),
        REVERSE({ -8.0 }),
        IDLE({ 0.0 });
    }

    init {
        RobotPeriodicManager.startPeriodic { trackState(); log() }

        motor.inverted = false
        motor.enableVoltageCompensation(11.0)
        motor.idleMode = CANSparkBase.IdleMode.kBrake
    }

    private fun trackState() {
        motor.setVoltage(state.getVoltage())
    }

    private fun log() {
        Logger.log("Loader/Motor", motor)
    }

    fun setState(state: State) {
        this.state = state
        trackState()
        Logger.log("Loader/State", state)
    }

    val isIdle get() = state == State.IDLE
}