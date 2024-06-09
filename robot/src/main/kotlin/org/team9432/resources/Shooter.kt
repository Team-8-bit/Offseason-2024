package org.team9432.resources

import edu.wpi.first.math.controller.PIDController
import kotlinx.coroutines.delay
import org.team9432.lib.resource.Action
import org.team9432.lib.resource.Resource
import org.team9432.lib.robot.CoroutineRobot
import org.team9432.lib.wrappers.Spark
import org.team9432.lib.wrappers.neo.LoggedNeo

object Shooter: Resource("Shooter") {
    private val topMotor = LoggedNeo(getConfig())
    private val bottomMotor = LoggedNeo(getConfig())

    val topPid = PIDController(0.0, 0.0, 0.0)
    val bottomPid = PIDController(0.0, 0.0, 0.0)

    var state = State.IDLE

    enum class State(val getTopVoltage: () -> Double, val getBottomVoltage: () -> Double) {
        IDLE({ 0.0 }, { 0.0 }),
        SHOOT({ 5000.0 }, { 5000.0 }),
        AMP({ 500.0 }, { 500.0 });
    }

    init {
        CoroutineRobot.addPeriodic {
            val topVoltage = topPid.calculate(topMotor.getAngle().rotations, state.getTopVoltage())
            val bottomVoltage = bottomPid.calculate(bottomMotor.getAngle().rotations, state.getBottomVoltage())

            topMotor.setVoltage(topVoltage)
            bottomMotor.setVoltage(bottomVoltage)
        }
    }

    override val defaultAction: Action = {
        state = State.IDLE
    }

    suspend fun spinUpToState(state: State) {
        Shooter.state = state
        while (!topPid.atSetpoint() && !bottomPid.atSetpoint()) delay(20)
    }

    private fun getConfig() = LoggedNeo.Config(
        canID = 0,
        deviceName = "Intake",
        gearRatio = 1.0,
        logName = "Intake",
        motorType = Spark.MotorType.NEO,
        simJkgMetersSquared = 0.003,
        sparkConfig = Spark.Config()
    )
}