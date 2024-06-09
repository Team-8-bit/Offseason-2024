package org.team9432.resources

import org.team9432.lib.resource.Action
import org.team9432.lib.resource.Resource
import org.team9432.lib.wrappers.Beambreak
import org.team9432.lib.wrappers.Spark
import org.team9432.lib.wrappers.neo.LoggedNeo

object Intake: Resource("Intake") {
    private val motor = LoggedNeo(getConfig())
    val initialSensor = Beambreak(0)

    var state = State.IDLE
        set(value) {
            motor.setVoltage(value.getVoltage())
            field = value
        }

    enum class State(val getVoltage: () -> Double) {
        INTAKE({ 5.0 }),
        OUTTAKE({ -5.0 }),
        IDLE({ 0.0 });
    }

    override val defaultAction: Action = {
        state = State.IDLE
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