package org.team9432.resources

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import org.team9432.lib.Beambreak
import org.team9432.lib.coroutines.CoroutineRobot
import org.team9432.lib.doglog.Logger
import org.team9432.lib.resource.Resource

object Intake: Resource("Intake") {
    private val leader = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    private val follower = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)

    private val beambreak = Beambreak(9)

    private var state = State.IDLE

    enum class State(val getVoltage: () -> Double) {
        INTAKE({ 5.0 }),
        OUTTAKE({ -5.0 }),
        IDLE({ 0.0 });
    }

    init {
        CoroutineRobot.startPeriodic { trackState(); log() }

        leader.inverted = true
        leader.enableVoltageCompensation(11.0)
        leader.idleMode = CANSparkBase.IdleMode.kBrake

        follower.inverted = true
        follower.enableVoltageCompensation(11.0)
        follower.idleMode = CANSparkBase.IdleMode.kBrake

        follower.follow(leader)
    }

    private fun trackState() {
        leader.setVoltage(state.getVoltage())
    }

    private fun log() {
        Logger.log("Intake/Leader", leader)
        Logger.log("Intake/Follower", follower)
        Logger.log("Intake/Beambreak", beambreak)
        Logger.log("Intake/State", state)
    }

    fun setState(state: State) {
        this.state = state
    }
}