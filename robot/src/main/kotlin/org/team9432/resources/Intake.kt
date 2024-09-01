package org.team9432.resources

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.doglog.Logger
import org.team9432.lib.resource.Resource

object Intake: Resource("Intake") {
    private val leader = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    private val follower = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)

    private var state = State.IDLE

    enum class State(val getVoltage: () -> Double) {
        INTAKE({ 10.0 }),
        LOAD({ 2.0 }),
        OUTTAKE({ -5.0 }),
        IDLE({ 0.0 });
    }

    init {
        RobotPeriodicManager.startPeriodic { trackState(); log() }

        leader.inverted = true
        leader.enableVoltageCompensation(11.0)
        leader.setSmartCurrentLimit(20)
        leader.idleMode = CANSparkBase.IdleMode.kBrake

        follower.inverted = true
        follower.enableVoltageCompensation(11.0)
        follower.setSmartCurrentLimit(20)
        follower.idleMode = CANSparkBase.IdleMode.kBrake

        follower.follow(leader)
    }

    private fun trackState() {
        leader.setVoltage(state.getVoltage())
    }

    private fun log() {
        Logger.log("Intake/Leader", leader)
        Logger.log("Intake/Follower", follower)
    }

    fun setState(state: State) {
        this.state = state
        trackState()
        Logger.log("Intake/State", Intake.state)
    }

    val isIntaking get() = state == State.INTAKE
    val isIdle get() = state == State.IDLE
}