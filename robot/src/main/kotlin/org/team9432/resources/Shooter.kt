package org.team9432.resources

import org.team9432.lib.resource.Resource
import org.team9432.lib.wrappers.Spark
import org.team9432.lib.wrappers.neo.LoggedNeo

object Shooter: Resource("Shooter") {
    private val topMotor = LoggedNeo(getConfig(14, "Top", false))
    private val bottomMotor = LoggedNeo(getConfig(13, "Bottom", true))

    private var state = State.IDLE

    enum class State(val topVoltage: Double, val bottomVoltage: Double) {
        IDLE(0.0, 0.0),
        SHOOT(11.0, 5.0),
        AMP(0.7, 4.6);
    }

    fun setState(state: State) {
        Shooter.state = state
        topMotor.setVoltage(state.topVoltage)
        bottomMotor.setVoltage(state.bottomVoltage)
    }

    private fun getConfig(canID: Int, additionalQualifier: String, inverted: Boolean) = LoggedNeo.Config(
        canID = canID,
        deviceName = "Shooter",
        gearRatio = 1.0,
        logName = "Shooter",
        additionalQualifier = additionalQualifier,
        motorType = Spark.MotorType.VORTEX,
        simJkgMetersSquared = 0.003,
        sparkConfig = Spark.Config(
            inverted = inverted
        )
    )
}