package org.team9432.resources

import org.team9432.lib.resource.Resource
import org.team9432.lib.robot.CoroutineRobot
import org.team9432.lib.util.enumValue
import org.team9432.lib.util.set
import org.team9432.lib.wrappers.Beambreak
import org.team9432.lib.wrappers.Spark
import org.team9432.lib.wrappers.neo.LoggedNeo

object Intake: Resource("Intake") {
    private val motor = LoggedNeo(getConfig())
    val initialSensor = Beambreak(0)

    private var state by table.enumValue("State", State.IDLE)

    init {
        CoroutineRobot.addPeriodic {
            table["Job"] = currentActionName ?: "null"
        }
    }

    enum class State(val getVoltage: () -> Double) {
        INTAKE({ 5.0 }),
        OUTTAKE({ -5.0 }),
        IDLE({ 0.0 });
    }

    fun set(state: State) {
        this.state = state
        motor.setVoltage(state.getVoltage())
    }

    private fun getConfig() = LoggedNeo.Config(
        canID = 1,
        deviceName = "Intake",
        gearRatio = 1.0,
        logName = "Intake",
        motorType = Spark.MotorType.NEO,
        simJkgMetersSquared = 0.003,
        sparkConfig = Spark.Config()
    )
}