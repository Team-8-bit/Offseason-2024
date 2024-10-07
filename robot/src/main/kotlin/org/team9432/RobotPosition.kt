package org.team9432

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.littletonrobotics.junction.AutoLogOutput
import org.team9432.lib.unit.Length
import org.team9432.lib.unit.asRotation2d
import org.team9432.lib.unit.inMeters
import org.team9432.lib.unit.meters
import org.team9432.lib.util.angleTo
import org.team9432.lib.util.distanceTo
import org.team9432.lib.util.transformBySpeeds
import org.team9432.lib.util.velocityLessThan
import org.team9432.resources.drive.DrivetrainConstants.DRIVE_KINEMATICS
import kotlin.math.abs
import kotlin.math.max

object RobotPosition {
    private val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        DRIVE_KINEMATICS,
        Rotation2d(),
        Array(4) { SwerveModulePosition() },
        Pose2d()
    )

    private var currentChassisSpeeds = ChassisSpeeds()
    private var previousVisionMeasurementTimeStamp: Double = -1.0

    fun applyOdometryObservation(currentTimeSeconds: Double, gyroRotation: Rotation2d, modulePositions: Array<SwerveModulePosition?>) {
        poseEstimator.updateWithTime(currentTimeSeconds, gyroRotation, modulePositions)
    }

    fun applyVisionMeasurement(visionPose: Pose2d, timestamp: Double, measurementStdDevs: Matrix<N3, N1>) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, measurementStdDevs)
        previousVisionMeasurementTimeStamp = max(timestamp, previousVisionMeasurementTimeStamp)
    }

    fun addVelocityData(velocity: ChassisSpeeds) {
        currentChassisSpeeds = velocity
    }

    @get:AutoLogOutput(key = "RobotPosition/CurrentPose")
    val currentPose: Pose2d get() = poseEstimator.estimatedPosition

    fun getFutureRobotPose(timeSeconds: Double) = currentPose.transformBySpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(currentChassisSpeeds, currentPose.rotation), timeSeconds)

    const val SHOT_TIME_SECONDS = 0.4

    fun getAngleToSpeaker() = getFutureRobotPose(SHOT_TIME_SECONDS).angleTo(PositionConstants.speakerAimPose).asRotation2d
    fun getCurrentSpeedsRobotRelative() = currentChassisSpeeds

    val isInScoringRangeOfSpeaker
        get() = distanceToSpeaker() < 2.3.meters &&
                distanceToSpeaker() > 1.3.meters

    /** Return the distance from the robot to the speaker. */
    fun distanceToSpeaker(): Length {
        return getFutureRobotPose(SHOT_TIME_SECONDS).distanceTo(PositionConstants.speakerAimPose)
    }

    fun isAimedAtSpeaker(): Boolean {
        return if (distanceToSpeaker().inMeters > 1.5) {
            getSpeakerAimingErrorDegrees() < 5
        } else if (distanceToSpeaker().inMeters > 1.25) {
            getSpeakerAimingErrorDegrees() < 10
        } else {
            getSpeakerAimingErrorDegrees() < 25
        }
    }

    private fun getSpeakerAimingErrorDegrees(): Double {
        return abs((getFutureRobotPose(SHOT_TIME_SECONDS).angleTo(PositionConstants.speakerAimPose).asRotation2d - currentPose.rotation).degrees)
    }

    fun resetOdometry(pose: Pose2d) = poseEstimator.resetPosition(pose.rotation, Array(4) { SwerveModulePosition() }, pose)

    fun velocityLessThan(metersPerSecond: Double, rotationsPerSecond: Double) = currentChassisSpeeds.velocityLessThan(metersPerSecond, rotationsPerSecond)
}