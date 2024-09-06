// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/
package org.team9432.resources.swerve

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.littletonrobotics.junction.Logger
import org.team9432.lib.RobotPeriodicManager
import org.team9432.resources.swerve.DriveTrainConstants.CHASSIS_MAX_ACCELERATION_MPS_SQ
import org.team9432.resources.swerve.DriveTrainConstants.CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ
import org.team9432.resources.swerve.DriveTrainConstants.CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC
import org.team9432.resources.swerve.DriveTrainConstants.CHASSIS_MAX_VELOCITY
import org.team9432.resources.swerve.DriveTrainConstants.DRIVE_KINEMATICS
import org.team9432.resources.swerve.DriveTrainConstants.MODULE_TRANSLATIONS
import org.team9432.resources.swerve.VisionConfigs.GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS
import org.team9432.resources.swerve.VisionConfigs.ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS
import org.team9432.resources.swerve.VisionConfigs.ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION
import org.team9432.resources.swerve.VisionConfigs.TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION
import org.team9432.resources.swerve.gyro.GyroIO
import org.team9432.resources.swerve.gyro.LoggedGyroIOInputs
import org.team9432.resources.swerve.module.ModuleIO
import org.team9432.resources.swerve.module.SwerveModule
import java.util.concurrent.locks.ReentrantLock
import kotlin.math.max

class SwerveDrive(private val gyroIO: GyroIO, frontLeftModuleIO: ModuleIO, frontRightModuleIO: ModuleIO, backLeftModuleIO: ModuleIO, backRightModuleIO: ModuleIO): HolonomicDriveSubsystem {
    companion object {
        val odometryLock = ReentrantLock()
    }

    private val gyroInputs = LoggedGyroIOInputs()
    private val odometryThreadInputs = LoggedOdometryThreadInputs()

    private val modules: Array<SwerveModule> = arrayOf(
        SwerveModule(frontLeftModuleIO, "FrontLeft"),
        SwerveModule(frontRightModuleIO, "FrontRight"),
        SwerveModule(backLeftModuleIO, "BackLeft"),
        SwerveModule(backRightModuleIO, "BackRight"),
    )

    private var rawGyroRotation: Rotation2d = Rotation2d()
    private val lastModulePositions: Array<SwerveModulePosition> = Array(4) { SwerveModulePosition() }

    private val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(
        DRIVE_KINEMATICS, rawGyroRotation, lastModulePositions, Pose2d(),
        VecBuilder.fill(ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS, ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS, GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS),
        VecBuilder.fill(
            TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION,
            TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION,
            ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION
        )
    )

    private val odometryThread = OdometryThread.createInstance()

    init {
        odometryThread.start()

        periodic()

        RobotPeriodicManager.startPeriodic {
            periodic()
            Logger.recordOutput("Odometry/Robot", getPose())
            Logger.recordOutput("Drive/MeasuredModuleStates", *moduleStates)
        }
    }

    private fun periodic() {
        odometryLock.lock()
        gyroIO.updateInputs(gyroInputs)
        modules.forEach(SwerveModule::updateInputs)
        odometryThread.updateInputs(odometryThreadInputs)
        odometryLock.unlock()

        Logger.processInputs("Drive/Gyro", gyroInputs)
        Logger.processInputs("Drive/OdometryThread", odometryThreadInputs)

        modules.forEach(SwerveModule::periodic)

        val timestampSamples = odometryThreadInputs.measurementTimestamps

        for (timestampIndex in timestampSamples.indices) {
            val modulePositions = arrayOfNulls<SwerveModulePosition>(4)
            val moduleDeltas = arrayOfNulls<SwerveModulePosition>(4)

            for (moduleIndex in modules.indices) {
                val modulePosition = modules[moduleIndex].odometryPositions[timestampIndex]
                modulePositions[moduleIndex] = modulePosition
                moduleDeltas[moduleIndex] = SwerveModulePosition(
                    modulePosition.distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                    modulePosition.angle
                )
                lastModulePositions[moduleIndex] = modulePosition
            }

            if (gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryYawPositions[timestampIndex]
            } else {
                val twist = DRIVE_KINEMATICS.toTwist2d(*moduleDeltas)
                rawGyroRotation += Rotation2d(twist.dtheta)
            }

            poseEstimator.updateWithTime(timestampSamples[timestampIndex], rawGyroRotation, modulePositions)
        }
    }

    override val rawGyroYaw: Rotation2d
        get() = gyroInputs.yawPosition

    override fun runRawChassisSpeeds(speeds: ChassisSpeeds) {
        val setpointStates: Array<SwerveModuleState> = DRIVE_KINEMATICS.toSwerveModuleStates(speeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, CHASSIS_MAX_VELOCITY)

        // Send setpoints to modules
        val optimizedSetpointStates = Array(4) { index -> modules[index].runSetpoint(setpointStates[index]) }

        Logger.recordOutput("Drive/Setpoints", *setpointStates)
        Logger.recordOutput("Drive/SetpointsOptimized", *optimizedSetpointStates)
    }

    /**
     * Locks the chassis and turns the modules to an X formation to resist movement.
     * The lock will be cancelled the next time a nonzero velocity is requested.
     */
    fun lockChassisWithXFormation() {
        val swerveHeadings = Array<Rotation2d>(modules.size) { index -> MODULE_TRANSLATIONS[index].angle }
        DRIVE_KINEMATICS.resetHeadings(*swerveHeadings)
        super.stop()
    }

    /** Returns the module states (turn angles and drive velocities) for all the modules. */
    private val moduleStates: Array<SwerveModuleState>
        get() = Array(modules.size) { index -> modules[index].measuredState }

    /** Returns the module positions (turn angles and drive positions) for all the modules. */
    private val moduleLatestPositions: Array<SwerveModulePosition?>
        get() = Array(modules.size) { index -> modules[index].latestPosition }

    override fun getPose() = poseEstimator.estimatedPosition
    override fun setPose(pose: Pose2d) = poseEstimator.resetPosition(rawGyroRotation, moduleLatestPositions, pose)

    override val measuredChassisSpeedsRobotRelative: ChassisSpeeds
        get() = DRIVE_KINEMATICS.toChassisSpeeds(*moduleStates)

    override val chassisMaxLinearVelocityMetersPerSec: Double
        get() = CHASSIS_MAX_VELOCITY
    override val chassisMaxAccelerationMetersPerSecSq: Double
        get() = CHASSIS_MAX_ACCELERATION_MPS_SQ
    override val chassisMaxAngularVelocity: Double
        get() = CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC
    override val chassisMaxAngularAccelerationRadPerSecSq: Double
        get() = CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ

    override fun addVisionMeasurement(visionPose: Pose2d?, timestamp: Double, measurementStdDevs: Matrix<N3?, N1?>?) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, measurementStdDevs)
        previousVisionMeasurementTimeStamp = max(timestamp, previousVisionMeasurementTimeStamp)
    }

    override var previousVisionMeasurementTimeStamp: Double = -1.0
        private set
}