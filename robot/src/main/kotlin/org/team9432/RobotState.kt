package org.team9432

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.Logger
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.unit.*
import org.team9432.lib.util.*
import org.team9432.resources.drive.DrivetrainConstants.DRIVE_KINEMATICS
import org.team9432.resources.flywheels.DifferentialFlywheelSpeedMap
import org.team9432.resources.flywheels.DifferentialFlywheelSpeedMap.ShooterSpeeds
import kotlin.math.max

object RobotState {
    private val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        DRIVE_KINEMATICS,
        Rotation2d(),
        Array(4) { SwerveModulePosition() },
        Pose2d()
    )

    private var currentChassisSpeeds = ChassisSpeeds()
    private var previousVisionMeasurementTimeStamp: Double = -1.0

    fun applyOdometryObservation(currentTimeSeconds: Double, gyroRotation: Rotation2d, modulePositions: Array<SwerveModulePosition?>) {
        latestAimingParameters.invalidate()
        poseEstimator.updateWithTime(currentTimeSeconds, gyroRotation, modulePositions)
    }

    fun applyVisionMeasurement(visionPose: Pose2d, timestamp: Double, measurementStdDevs: Matrix<N3, N1>) {
        latestAimingParameters.invalidate()
        poseEstimator.addVisionMeasurement(visionPose, timestamp, measurementStdDevs)
        previousVisionMeasurementTimeStamp = max(timestamp, previousVisionMeasurementTimeStamp)
    }

    fun addVelocityData(velocity: ChassisSpeeds) {
        latestAimingParameters.invalidate()
        currentChassisSpeeds = velocity
    }

    val currentPose: Pose2d get() = poseEstimator.estimatedPosition

    data class AimingParameters(val shooterSpeeds: ShooterSpeeds, val drivetrainAngle: Rotation2d, val pivotAngle: Angle)

    private var latestAimingParameters = CachedValue<AimingParameters>()

    var shouldDisableShootOnMove = { false }
    var shouldUsePivotSetpoints = { true }

    fun getStandardAimingParameters(): AimingParameters {
        // Return the latest aiming parameters if they haven't changed
        latestAimingParameters.ifValid { cachedValue -> return cachedValue }

        val predictedFuturePose = if (!shouldDisableShootOnMove()) getFutureRobotPose(SHOT_TIME_SECONDS) else currentPose
        val targetPose = PositionConstants.speakerAimPose

        val drivetrainAngleTarget = predictedFuturePose.angleTo(targetPose).asRotation2d
        val distanceToSpeaker = predictedFuturePose.distanceTo(targetPose)

        val pivotAngleTarget = pivotAngleMap.get(distanceToSpeaker.inMeters)
        val shooterSpeedTarget = if (shouldUsePivotSetpoints.invoke() && distanceToSpeaker > 1.5) {
            ShooterSpeeds(upperRPM = 5000.0, lowerRPM = 5000.0)
        } else {
            differentialShooterSpeedsMap.getMapValue(distanceToSpeaker)
        }

        return AimingParameters(
            shooterSpeeds = shooterSpeedTarget,
            drivetrainAngle = drivetrainAngleTarget,
            pivotAngle = pivotAngleTarget.degrees
        )
    }

    val swerveLimits = SwerveSetpointGenerator.ModuleLimits(
        maxDriveVelocity = 4.0,
        maxDriveAcceleration = 20.0,
        maxSteeringVelocity = Units.degreesToRadians(1080.0)
    )

    private val differentialShooterSpeedsMap = DifferentialFlywheelSpeedMap().apply {
        addMapValue(2.25.meters, ShooterSpeeds(upperRPM = 5000.0, lowerRPM = 2000.0))
        addMapValue(2.0.meters, ShooterSpeeds(upperRPM = 5000.0, lowerRPM = 2800.0))
        addMapValue(1.75.meters, ShooterSpeeds(upperRPM = 4000.0, lowerRPM = 3000.0))
        addMapValue(1.5.meters, ShooterSpeeds(upperRPM = 4000.0, lowerRPM = 4000.0))
        addMapValue(1.2.meters, ShooterSpeeds(upperRPM = 4500.0, lowerRPM = 3000.0))
    }

    private val pivotAngleMap = InterpolatingDoubleTreeMap().apply {
        // Meters to Degrees
        put(1.5, 0.0)
        put(2.0, 8.0)
        put(2.5, 15.0)
        put(3.0, 18.0)
        put(3.5, 22.0)
        put(4.0, 25.0)
    }

    private fun getFutureRobotPose(timeSeconds: Double) = currentPose.transformBySpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(currentChassisSpeeds, currentPose.rotation), timeSeconds)

    private const val SHOT_TIME_SECONDS = 0.4

    val isInSpeakerScoringRange: Boolean
        get() {
            val distance = currentPose.distanceTo(PositionConstants.speakerAimPose)
            return if (shouldUsePivotSetpoints()) {
                distance > 1.meters && distance < 4.0.meters
            } else {
                distance > 1.meters && distance < 2.5.meters
            }
        }

    val isInSpeakerPrepareRange: Boolean
        get() = currentPose.distanceTo(PositionConstants.speakerAimPose) < 10.0.meters

    fun resetOdometry(pose: Pose2d, rawGyro: Rotation2d, modulePositions: Array<SwerveModulePosition>) {
        poseEstimator.resetPosition(rawGyro, modulePositions, pose)
    }

    fun getRobotRelativeChassisSpeeds() = currentChassisSpeeds

    init {
        RobotPeriodicManager.startPeriodic {
            Logger.recordOutput("RobotPosition/CurrentPose", currentPose)
            Logger.recordOutput("RobotPosition/CurrentSpeeds", getRobotRelativeChassisSpeeds())
            Logger.recordOutput("RobotPosition/CurrentFieldRelativeSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), currentPose.rotation))
            Logger.recordOutput("RobotPosition/PoseInShotTime", getFutureRobotPose(SHOT_TIME_SECONDS))
            Logger.recordOutput("RobotPosition/SpeakerTuningDistanceMeters", currentPose.distanceTo(PositionConstants.speakerAimPose).inMeters)
        }
    }
}