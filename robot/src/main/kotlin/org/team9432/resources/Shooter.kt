package org.team9432.resources

import edu.wpi.first.math.controller.PIDController
import kotlinx.coroutines.delay
import org.team9432.lib.resource.Action
import org.team9432.lib.resource.Resource
import org.team9432.lib.robot.CoroutineRobot
import org.team9432.lib.wrappers.Spark
import org.team9432.lib.wrappers.neo.LoggedNeo

object Shooter: Resource("Shooter") {
    private val topMotor = LoggedNeo(getConfig(14, "Top"))
    private val bottomMotor = LoggedNeo(getConfig(13, "Bottom"))

    private val topPid = PIDController(0.0, 0.0, 0.0)
    private val bottomPid = PIDController(0.0, 0.0, 0.0)

    private var state = State.IDLE

    enum class State(val getTopVoltage: () -> Double, val getBottomVoltage: () -> Double) {
        IDLE({ 0.0 }, { 0.0 }),
        SHOOT({ 11.0 }, { -5.0 }),
        AMP({ 2.5 }, { -5.0 });
    }

    init {
        CoroutineRobot.addPeriodic {
            val topVoltage = topPid.calculate(topMotor.getAngle().rotations, state.getTopVoltage())
            val bottomVoltage = bottomPid.calculate(bottomMotor.getAngle().rotations, state.getBottomVoltage())

//            topMotor.setVoltage(topVoltage)
//            bottomMotor.setVoltage(bottomVoltage)
        }
    }

//    override val defaultAction: Action = {
//        state = State.IDLE
//    }

    suspend fun spinUpToState(state: State) {
        Shooter.state = state
        topMotor.setVoltage(state.getTopVoltage())
        bottomMotor.setVoltage(state.getBottomVoltage())
//        while (!topPid.atSetpoint() && !bottomPid.atSetpoint()) delay(20)
    }

    private fun getConfig(canID: Int, additionalQualifier: String) = LoggedNeo.Config(
        canID = canID,
        deviceName = "Shooter",
        gearRatio = 1.0,
        logName = "Shooter",
        additionalQualifier = additionalQualifier,
        motorType = Spark.MotorType.VORTEX,
        simJkgMetersSquared = 0.003,
        sparkConfig = Spark.Config(
            inverted = false
        )
    )
}