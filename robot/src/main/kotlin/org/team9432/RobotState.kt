package org.team9432

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import org.littletonrobotics.junction.Logger
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.constants.EvergreenFieldConstants
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
    private var latestDemoTagPose: Pose3d? = null

    fun applyOdometryObservation(currentTimeSeconds: Double, gyroRotation: Rotation2d, modulePositions: Array<SwerveModulePosition?>) {
        latestAimingParameters.invalidate()
        poseEstimator.updateWithTime(currentTimeSeconds, gyroRotation, modulePositions)
    }

    fun applyVisionMeasurement(visionPose: Pose2d, timestamp: Double, measurementStdDevs: Matrix<N3, N1>) {
        latestAimingParameters.invalidate()
        poseEstimator.addVisionMeasurement(visionPose, timestamp, measurementStdDevs)
        previousVisionMeasurementTimeStamp = max(timestamp, previousVisionMeasurementTimeStamp)

        Logger.recordOutput("RobotPosition/LatestVisionPose", visionPose)
        Logger.recordOutput("RobotPosition/LatestVisionStddevsXY", measurementStdDevs.get(0, 0))
        Logger.recordOutput("RobotPosition/LatestVisionStddevsRotation", measurementStdDevs.get(2, 0))
    }

    fun addVelocityData(velocity: ChassisSpeeds) {
        latestAimingParameters.invalidate()
        currentChassisSpeeds = velocity
    }

    fun applyDemoTagPose(tagPose: Pose3d?) {
        if (tagPose != null) latestDemoAimingParameters.invalidate()
        latestDemoTagPose = tagPose
    }

    val currentPose: Pose2d get() = poseEstimator.estimatedPosition

    data class AimingParameters(val shooterSpeeds: ShooterSpeeds, val drivetrainAngle: Rotation2d, val pivotAngle: Angle)

    private var latestAimingParameters = CachedValue<AimingParameters>()
    private var latestFeedAimingParameters = CachedValue<AimingParameters>()
    private var latestDemoAimingParameters = CachedValue<AimingParameters>()

    var shouldDisableShootOnMove = { false }
    var shouldUsePivotSetpoints = { true }

    // Aims as if the robot was in this pose during auto, useful for being ready at the end of a trajectory
    var autoPivotAimFromPose: Pose2d? = null
        set(value) {
            latestAimingParameters.invalidate()
            field = value
        }

    fun getDemoAimingParameters(): AimingParameters {
        // Return the latest aiming parameters if they haven't changed
        latestDemoAimingParameters.ifValid { cachedValue -> return cachedValue }

        val robotPose = currentPose

        val targetPose = latestDemoTagPose ?: return latestDemoAimingParameters.value ?: AimingParameters(ShooterSpeeds(0.0, 0.0), robotPose.rotation, 0.0.degrees)
        val targetPose2d = targetPose.toPose2d()

        val drivetrainAngleTarget = robotPose.angleTo(targetPose2d).asRotation2d

        val basePivotPosition = Translation2d(
            Units.inchesToMeters(-1.0),
            Units.inchesToMeters(15.5)
        )

        val tagTargetPosition = Translation2d(targetPose.x, targetPose.z)

        val angle = basePivotPosition.angleTo(tagTargetPosition)

        Logger.recordOutput("DemoShot/PivotAngle", angle.inDegrees + 50)

        val mechanism = Mechanism2d(3.0, 3.0)
        mechanism.getRoot("Base", basePivotPosition.x, basePivotPosition.y).append(MechanismLigament2d("Arm", 0.0, 5.0))
        mechanism.getRoot("Tag", tagTargetPosition.x, tagTargetPosition.y).append(MechanismLigament2d("Arm2", 0.0, 5.0))

        Logger.recordOutput("DemoShot/Mechanism", mechanism)

        val output = AimingParameters(
            shooterSpeeds = ShooterSpeeds(5000.0, 5000.0),
            drivetrainAngle = drivetrainAngleTarget,
            pivotAngle = 50.0.degrees - angle
        )

        latestDemoAimingParameters.value = output
        return output
    }

    fun getStandardAimingParameters(): AimingParameters {
        // Return the latest aiming parameters if they haven't changed
        latestAimingParameters.ifValid { cachedValue -> return cachedValue }

        val robotPose = if (!shouldDisableShootOnMove()) {
            getFutureRobotPose()
        } else {
            currentPose
        }

        val targetPose = PositionConstants.speakerAimPose

        val drivetrainAngleTarget = robotPose.angleTo(targetPose).asRotation2d
        val distanceToSpeaker = robotPose.distanceTo(targetPose)

        val finalPivotAutoAimFromPose = autoPivotAimFromPose
        val pivotAngleTarget = if (Robot.isAutonomousEnabled && finalPivotAutoAimFromPose != null) {
            pivotAngleMap.get(finalPivotAutoAimFromPose.distanceTo(targetPose).inMeters)
        } else {
            pivotAngleMap.get(distanceToSpeaker.inMeters)
        }

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

    fun getFeedAimingParameters(): AimingParameters {
        // Return the latest aiming parameters if they haven't changed
        latestFeedAimingParameters.ifValid { cachedValue -> return cachedValue }

        val robotPose = if (!shouldDisableShootOnMove()) {
            getFutureRobotPose(lookaheadTime = 1.1)
        } else {
            currentPose
        }

        val shouldToss = currentPose.y > ((EvergreenFieldConstants.lengthY / 3) * 2)

        val targetPose = if (shouldToss) {
            Translation2d(0.0.meters, EvergreenFieldConstants.lengthY - 0.25.meters).applyFlip()
        } else {
            Translation2d(0.0.meters, EvergreenFieldConstants.lengthY - 0.5.meters).applyFlip()
        }

        val drivetrainAngleTarget = robotPose.angleTo(targetPose).asRotation2d
        val distanceToTarget = robotPose.distanceTo(targetPose)

        Logger.recordOutput("RobotPosition/FeedTarget", targetPose)
        Logger.recordOutput("RobotPosition/FeedTargetDistance", distanceToTarget.inMeters)

        val pivotAngleTarget: Double
        val shooterSpeedTarget: ShooterSpeeds
        if (shouldToss) {
            pivotAngleTarget = 26.0 // Toss
            shooterSpeedTarget = ShooterSpeeds(3500.0, 3500.0)
        } else {
            pivotAngleTarget = pivotFeedMap.get(distanceToTarget.inMeters) // Over stage
            shooterSpeedTarget = feedSpeeds.getMapValue(distanceToTarget)
        }

        return AimingParameters(
            shooterSpeeds = shooterSpeedTarget,
            drivetrainAngle = drivetrainAngleTarget,
            pivotAngle = pivotAngleTarget.degrees
        )
    }

    val swerveLimits = SwerveSetpointGenerator.ModuleLimits(
        maxDriveVelocity = 4.0,
        maxDriveAcceleration = 10.0,
        maxSteeringVelocity = Units.degreesToRadians(1080.0)
    )

    private val feedSpeeds = DifferentialFlywheelSpeedMap().apply {
        addMapValue(10.0.meters, ShooterSpeeds(upperRPM = 4000.0, lowerRPM = 4800.0))
        addMapValue(12.meters, ShooterSpeeds(upperRPM = 3500.0, lowerRPM = 3500.0))
    }

    private val pivotFeedMap = InterpolatingDoubleTreeMap().apply {
        // Meters to corner to Degrees
        put(10.0, 0.0)
        put(12.0, 7.0)
    }

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

    private val shotTimeMap = InterpolatingDoubleTreeMap().apply {
        put(2.0, 0.4)
        put(4.0, 0.6)
    }

    private fun getFutureRobotPose(lookaheadTime: Double? = null): Pose2d {
        val distance = currentPose.distanceTo(PositionConstants.speakerAimPose)
        return currentPose.transformBySpeeds(
            ChassisSpeeds.fromRobotRelativeSpeeds(currentChassisSpeeds, currentPose.rotation),
            lookaheadTime ?: shotTimeMap.get(distance.inMeters)
        )
    }

    val isInSpeakerScoringRange: Boolean
        get() {
            val distance = currentPose.distanceTo(PositionConstants.speakerAimPose)
            return if (shouldUsePivotSetpoints()) {
                distance > 1.meters && distance < 5.0.meters
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
            Logger.recordOutput("RobotPosition/AutoPivotAimFromPose", *listOfNotNull(autoPivotAimFromPose).toTypedArray())
            Logger.recordOutput("RobotPosition/CurrentFieldRelativeSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), currentPose.rotation))
            Logger.recordOutput("RobotPosition/PoseInShotTime", getFutureRobotPose())
            Logger.recordOutput("RobotPosition/SpeakerTuningDistanceMeters", currentPose.distanceTo(PositionConstants.speakerAimPose).inMeters)
        }
    }
}