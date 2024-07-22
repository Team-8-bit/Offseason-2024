package org.team9432.resources

import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import org.team9432.lib.resource.Resource
import org.team9432.lib.robot.CoroutineRobot
import org.team9432.lib.util.enumValue
import org.team9432.lib.util.set

object Intake: Resource("Intake") {
    private val leader = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    private val follower = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)

    private var state by table.enumValue("State", State.IDLE)

    init {
        CoroutineRobot.startPeriodic {
            table["Job"] = currentActionName ?: "null"
        }

        leader.inverted = true
        follower.inverted = true
        follower.follow(leader)
    }

    enum class State(val getVoltage: () -> Double) {
        INTAKE({ 5.0 }),
        OUTTAKE({ -5.0 }),
        IDLE({ 0.0 });
    }

    fun set(state: State) {
        this.state = state
        leader.setVoltage(state.getVoltage())
    }
}