package org.team9432.resources.swerve.mapleswerve.utils.CustomPIDs

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile

class MapleProfiledPIDController(private val pidConfig: MaplePIDController.MaplePIDConfig, constraints: TrapezoidProfile.Constraints?):
    ProfiledPIDController(pidConfig.Kp, pidConfig.Ki, pidConfig.Kd, constraints) {
    override fun calculate(measurement: Double): Double {
        return MathUtil.clamp(super.calculate(measurement), -pidConfig.maximumPower, pidConfig.maximumPower)
    }
}
