package org.team9432.resources.swerve

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.team9432.resources.swerve.JoystickConfigs.ANGULAR_ACCELERATION_SMOOTH_OUT_SECONDS
import org.team9432.resources.swerve.JoystickConfigs.LINEAR_ACCELERATION_SMOOTH_OUT_SECONDS
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.withSign

interface HolonomicDriveSubsystem {
    /**
     * runs a ChassisSpeeds without doing any pre-processing
     * @param speeds a discrete chassis speed, robot-centric
     */
    fun runRawChassisSpeeds(speeds: ChassisSpeeds)

    /**
     * Returns the current odometry Pose.
     */
    fun getPose(): Pose2d
    /**
     * Resets the current odometry Pose to a given Pose
     */
    fun setPose(pose: Pose2d)


    val facing: Rotation2d
        get() = getPose().rotation

    val rawGyroYaw: Rotation2d
        get() = facing

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp  The timestamp of the vision measurement in seconds.
     * @param measurementStdDevs the standard deviation of the measurement
     */
    fun addVisionMeasurement(visionPose: Pose2d, timestamp: Double, measurementStdDevs: Matrix<N3, N1>) {}

    val previousVisionMeasurementTimeStamp: Double
        get() = (-1).toDouble()

    /**
     * @return the measured(actual) velocities of the chassis, robot-relative
     */
    val measuredChassisSpeedsRobotRelative: ChassisSpeeds?

    val measuredChassisSpeedsFieldRelative: ChassisSpeeds
        get() = ChassisSpeeds.fromRobotRelativeSpeeds(measuredChassisSpeedsRobotRelative, facing)

    val chassisMaxLinearVelocityMetersPerSec: Double
    val chassisMaxAccelerationMetersPerSecSq: Double
    val chassisMaxAngularVelocity: Double
    val chassisMaxAngularAccelerationRadPerSecSq: Double

    /**
     * runs a driverstation-centric ChassisSpeeds
     * @param driverStationCentricSpeeds a continuous chassis speeds, driverstation-centric, normally from a gamepad
     */
    fun runDriverStationCentricChassisSpeeds(driverStationCentricSpeeds: ChassisSpeeds) {
        val driverStationFacing: Rotation2d = FieldConstants.driverStationFacing
        runRobotCentricChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                driverStationCentricSpeeds,
                getPose().rotation.minus(driverStationFacing)
            )
        )
    }

    /**
     * runs a field-centric ChassisSpeeds
     * @param fieldCentricSpeeds a continuous chassis speeds, field-centric, normally from a pid position controller
     */
    fun runFieldCentricChassisSpeeds(fieldCentricSpeeds: ChassisSpeeds) {
        runRobotCentricChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldCentricSpeeds,
                getPose().rotation
            )
        )
    }



    fun stop() {
        runRobotCentricChassisSpeeds(ChassisSpeeds())
    }

    /**
     * runs a ChassisSpeeds, pre-processed with ChassisSpeeds.discretize()
     * @param speeds a continuous chassis speed, robot-centric
     */
    fun runRobotCentricChassisSpeeds(speeds: ChassisSpeeds) {
        var speeds = speeds
        val PERCENT_DEADBAND = 0.03
        if (abs(speeds.omegaRadiansPerSecond) < PERCENT_DEADBAND * chassisMaxAngularVelocity
            && hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < PERCENT_DEADBAND * chassisMaxLinearVelocityMetersPerSec) speeds = ChassisSpeeds()

        runRawChassisSpeeds(ChassisSpeeds.discretize(speeds, 0.02))
    }

    fun constrainAcceleration(
        currentSpeeds: ChassisSpeeds, desiredSpeeds: ChassisSpeeds,
        dtSecs: Double,
    ): ChassisSpeeds {
        val MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ: Double = (chassisMaxLinearVelocityMetersPerSec
                / LINEAR_ACCELERATION_SMOOTH_OUT_SECONDS)
        val MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ: Double = (chassisMaxAngularVelocity
                / ANGULAR_ACCELERATION_SMOOTH_OUT_SECONDS)

        val currentLinearVelocityMetersPerSec = Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
        val desiredLinearVelocityMetersPerSec = Translation2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond)
        val linearVelocityDifference = desiredLinearVelocityMetersPerSec.minus(currentLinearVelocityMetersPerSec)

        val maxLinearVelocityChangeIn1Period = MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ * dtSecs
        val desiredLinearVelocityReachableWithin1Period = linearVelocityDifference.norm <= maxLinearVelocityChangeIn1Period
        val linearVelocityChangeVector = Translation2d(maxLinearVelocityChangeIn1Period, linearVelocityDifference.angle)
        val newLinearVelocity = if (desiredLinearVelocityReachableWithin1Period)
            desiredLinearVelocityMetersPerSec
        else
            currentLinearVelocityMetersPerSec.plus(linearVelocityChangeVector)

        val angularVelocityDifference = desiredSpeeds.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond
        val maxAngularVelocityChangeIn1Period = MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ * dtSecs
        val angularVelocityChange = maxAngularVelocityChangeIn1Period.withSign(angularVelocityDifference)
        val desiredAngularVelocityReachableWithin1Period = abs(angularVelocityDifference) <= maxAngularVelocityChangeIn1Period
        val newAngularVelocity = if (desiredAngularVelocityReachableWithin1Period)
            desiredSpeeds.omegaRadiansPerSecond
        else
            currentSpeeds.omegaRadiansPerSecond + angularVelocityChange
        return ChassisSpeeds(
            newLinearVelocity.x,
            newLinearVelocity.y,
            newAngularVelocity
        )
    }

    companion object {
        fun isZero(chassisSpeeds: ChassisSpeeds): Boolean {
            return abs(chassisSpeeds.omegaRadiansPerSecond) < Math.toRadians(5.0) && abs(chassisSpeeds.vxMetersPerSecond) < 0.05 && abs(chassisSpeeds.vyMetersPerSecond) < 0.05
        }
    }
}