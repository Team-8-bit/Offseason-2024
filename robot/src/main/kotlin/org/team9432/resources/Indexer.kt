package org.team9432.resources

import org.team9432.lib.resource.Resource
import org.team9432.lib.util.enumValue
import org.team9432.lib.wrappers.Beambreak
import org.team9432.lib.wrappers.Spark
import org.team9432.lib.wrappers.neo.LoggedNeo

object Indexer: Resource("Indexer") {
    private val motor = LoggedNeo(getConfig())
    val sensor = Beambreak(1)

    private var state by table.enumValue("State", State.IDLE)

    init {
        table.getBooleanTopic("Beambreak").publish()
    }

    enum class State(val getVoltage: () -> Double) {
        LOAD({ 5.0 }),
        REVERSE({ -5.0 }),
        IDLE({ 0.0 });
    }

    fun set(state: State) {
        this.state = state
        motor.setVoltage(state.getVoltage())
    }

    private fun getConfig() = LoggedNeo.Config(
        canID = 2,
        deviceName = "Indexer",
        gearRatio = 1.0,
        logName = "Indexer",
        motorType = Spark.MotorType.NEO,
        simJkgMetersSquared = 0.003,
        sparkConfig = Spark.Config()
    )
}