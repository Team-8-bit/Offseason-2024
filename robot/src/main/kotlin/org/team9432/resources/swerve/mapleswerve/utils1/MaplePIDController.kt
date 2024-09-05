package org.team9432.resources.swerve.mapleswerve.utils1

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.util.Units

class MaplePIDController(
    config: MaplePIDConfig,
): PIDController(config.Kp, config.Ki, config.Kd) {
    private val config: MaplePIDConfig

    init {
        if (config.isCircularLoop) super.enableContinuousInput(0.0, Units.rotationsToRadians(1.0))
        super.setTolerance(config.errorTolerance)
        this.config = config
    }

    override fun calculate(measurement: Double): Double {
        return MathUtil.clamp(
            MathUtil.applyDeadband(super.calculate(measurement), config.deadBand),
            -config.maximumPower, config.maximumPower
        )
    }

    class MaplePIDConfig(
        val maximumPower: Double,
        val errorStartDecelerate: Double,
        percentDeadBand: Double,
        val errorTolerance: Double,
        val timeThinkAhead: Double,
        val isCircularLoop: Boolean,
        val Ki: Double,
    ) {
        val deadBand: Double = percentDeadBand * maximumPower

        val Kp: Double = maximumPower / errorStartDecelerate
        val Kd: Double = Kp * timeThinkAhead
    }
}