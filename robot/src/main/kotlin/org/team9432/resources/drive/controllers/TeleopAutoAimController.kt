package org.team9432.resources.drive.controllers

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.Logger
import org.team9432.RobotState
import org.team9432.lib.dashboard.LoggedTunableNumber
import org.team9432.lib.util.epsilonEquals
import org.team9432.resources.drive.DrivetrainConstants
import kotlin.math.abs

class TeleopAutoAimController(private val goalSupplier: () -> Rotation2d, private val toleranceSupplierDegrees: () -> Double): GenericDriveController<Double>() {
    private val controller = ProfiledPIDController(0.0, 0.0, 0.0, TrapezoidProfile.Constraints(0.0, 0.0)).apply {
        enableContinuousInput(-Math.PI, Math.PI)

        reset(
            RobotState.currentPose.rotation.radians,
            RobotState.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond
        )
    }

    companion object {
        private const val TABLE_KEY = "TeleopAutoAimController"

        private val kP by LoggedTunableNumber("$TABLE_KEY/kP", 6.0)
        private val kD by LoggedTunableNumber("$TABLE_KEY/kD", 0.3)
        private val maxVelocityMultiplier by LoggedTunableNumber("$TABLE_KEY/MaxVelocityPercent", 0.8)
        private val maxAccelerationMultiplier by LoggedTunableNumber("$TABLE_KEY/MaxAccelerationPercent", 0.7)
    }

    override fun calculate(): Double {
        controller.setPID(kP, 0.0, kD)
        controller.setTolerance(Units.degreesToRadians(toleranceSupplierDegrees.invoke()))

        val maxAngularVelocity = (RobotState.swerveLimits.maxDriveVelocity / DrivetrainConstants.DRIVE_BASE_RADIUS) * maxVelocityMultiplier
        val maxAngularAcceleration = (RobotState.swerveLimits.maxDriveAcceleration / DrivetrainConstants.DRIVE_BASE_RADIUS) * maxAccelerationMultiplier
        controller.constraints = TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration)

        val output = controller.calculate(
            RobotState.currentPose.rotation.radians,
            goalSupplier.invoke().radians
        )

        Logger.recordOutput("$TABLE_KEY/PositionErrorDegrees", Units.radiansToDegrees(controller.positionError))
        Logger.recordOutput("$TABLE_KEY/AtGoal", atGoal())

        return output
    }

    fun atGoal(toleranceDegrees: Double) =
        abs(Units.radiansToDegrees(controller.positionError)) < toleranceDegrees
//        epsilonEquals(
//            controller.setpoint.position,
//            controller.goal.position,
//            Units.degreesToRadians(toleranceDegrees)
//        )

    override fun atGoal(): Boolean {
        return atGoal(Units.radiansToDegrees(controller.positionTolerance))
    }
}