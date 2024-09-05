// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/
package org.team9432.resources.swerve

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team9432.lib.RobotPeriodicManager
import org.team9432.resources.swerve.DriveTrainConstants.CHASSIS_MAX_ACCELERATION_MPS_SQ
import org.team9432.resources.swerve.DriveTrainConstants.CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ
import org.team9432.resources.swerve.DriveTrainConstants.CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC
import org.team9432.resources.swerve.DriveTrainConstants.CHASSIS_MAX_VELOCITY
import org.team9432.resources.swerve.DriveTrainConstants.DRIVE_KINEMATICS
import org.team9432.resources.swerve.VisionConfigs.GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS
import org.team9432.resources.swerve.VisionConfigs.ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS
import org.team9432.resources.swerve.VisionConfigs.ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION
import org.team9432.resources.swerve.VisionConfigs.TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION
import org.team9432.resources.swerve.gyro.GyroIO
import org.team9432.resources.swerve.gyro.LoggedGyroIOInputs
import org.team9432.resources.swerve.mapleswerve.MapleTimeUtils
import org.team9432.resources.swerve.module.ModuleIO
import org.team9432.resources.swerve.module.SwerveModule
import kotlin.math.max

class SwerveDrive(type: DriveType, gyroIO: GyroIO, frontLeftModuleIO: ModuleIO, frontRightModuleIO: ModuleIO, backLeftModuleIO: ModuleIO, backRightModuleIO: ModuleIO): HolonomicDriveSubsystem {
    enum class DriveType {
        GENERIC,
        CTRE_ON_RIO,
        CTRE_ON_CANIVORE
    }

    private val gyroIO: GyroIO = gyroIO
    private val gyroInputs: LoggedGyroIOInputs
    private val odometryThreadInputs: LoggedOdometryThreadInputs
    private val swerveModules: Array<SwerveModule>

    private var rawGyroRotation: Rotation2d
    private val lastModulePositions: Array<SwerveModulePosition?>
    private val poseEstimator: SwerveDrivePoseEstimator

    private val odometryThread: OdometryThread
    fun periodic() {
        val t0: Double = MapleTimeUtils.realTimeSeconds
        fetchOdometryInputs()
        Logger.recordOutput("SystemPerformance/OdometryFetchingTimeMS", (MapleTimeUtils.realTimeSeconds - t0) * 1000)
        modulesPeriodic()

        for (timeStampIndex in 0 until odometryThreadInputs.measurementTimeStamps.size) feedSingleOdometryDataToPositionEstimator(timeStampIndex)
    }

    private fun fetchOdometryInputs() {
        odometryThread.lockOdometry()
        odometryThread.updateInputs(odometryThreadInputs)
        Logger.processInputs("Drive/OdometryThread", odometryThreadInputs)

        for (module in swerveModules) module.updateOdometryInputs()

        gyroIO.updateInputs(gyroInputs)
        Logger.processInputs("Drive/Gyro", gyroInputs)

        odometryThread.unlockOdometry()
    }

    private fun modulesPeriodic() {
        for (module in swerveModules) module.periodic()
    }

    private fun feedSingleOdometryDataToPositionEstimator(timeStampIndex: Int) {
        val modulePositions = getModulesPosition(timeStampIndex)
        val moduleDeltas = getModulesDelta(modulePositions)

        if (!updateRobotFacingWithGyroReading(timeStampIndex)) updateRobotFacingWithOdometry(moduleDeltas)

        poseEstimator.updateWithTime(
            odometryThreadInputs.measurementTimeStamps[timeStampIndex],
            rawGyroRotation,
            modulePositions
        )
    }

    private fun getModulesPosition(timeStampIndex: Int): Array<SwerveModulePosition?> {
        val swerveModulePositions = arrayOfNulls<SwerveModulePosition>(swerveModules.size)
        for (moduleIndex in 0..3) swerveModulePositions[moduleIndex] = swerveModules[moduleIndex].odometryPositions.get(timeStampIndex)
        return swerveModulePositions
    }

    private fun getModulesDelta(freshModulesPosition: Array<SwerveModulePosition?>): Array<SwerveModulePosition?> {
        val deltas = arrayOfNulls<SwerveModulePosition>(swerveModules.size)
        for (moduleIndex in 0..3) {
            val deltaDistanceMeters = (freshModulesPosition[moduleIndex]!!.distanceMeters
                    - lastModulePositions[moduleIndex]!!.distanceMeters)
            deltas[moduleIndex] = SwerveModulePosition(deltaDistanceMeters, freshModulesPosition[moduleIndex]!!.angle)
            lastModulePositions[moduleIndex] = freshModulesPosition[moduleIndex]
        }
        return deltas
    }

    /**
     * updates the robot facing using the reading from the gyro
     * @param timeStampIndex the index of the time stamp
     * @return whether the update is success
     */
    private fun updateRobotFacingWithGyroReading(timeStampIndex: Int): Boolean {
        if (!gyroInputs.connected) return false
        rawGyroRotation = gyroInputs.odometryYawPositions.get(timeStampIndex)
        return true
    }

    /**
     * updates the robot facing using the reading from the gyro
     * @param modulesDelta the delta of the swerve modules calculated from the odometry
     */
    private fun updateRobotFacingWithOdometry(modulesDelta: Array<SwerveModulePosition?>) {
        if (modulesDelta.any { it == null }) return
        val twist: Twist2d = DRIVE_KINEMATICS.toTwist2d(*modulesDelta)
        rawGyroRotation = rawGyroRotation.plus(Rotation2d(twist.dtheta))
    }

    override fun runRawChassisSpeeds(speeds: ChassisSpeeds) {
        val setpointStates: Array<SwerveModuleState> = DRIVE_KINEMATICS.toSwerveModuleStates(speeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, CHASSIS_MAX_VELOCITY)

        // Send setpoints to modules
        val optimizedSetpointStates = arrayOfNulls<SwerveModuleState>(4)
        for (i in 0..3) optimizedSetpointStates[i] = swerveModules[i].runSetPoint(setpointStates[i])

        Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", *optimizedSetpointStates)
    }

//    /**
//     * Locks the chassis and turns the modules to an X formation to resist movement.
//     * The lock will be cancelled the next time a nonzero velocity is requested.
//     */
//    fun lockChassisWithXFormation() {
//        val swerveHeadings = arrayOfNulls<Rotation2d>(swerveModules.size)
//        for (i in swerveHeadings.indices) swerveHeadings[i] = MODULE_TRANSLATIONS.get(i).getAngle()
//        DRIVE_KINEMATICS.resetHeadings(swerveHeadings)
//        super.stop()
//    }

    @get:AutoLogOutput(key = "SwerveStates/Measured")
    private val moduleStates: Array<SwerveModuleState?>
        /**
         * Returns the module states (turn angles and drive velocities) for all the modules.
         */
        get() {
            val states = arrayOfNulls<SwerveModuleState>(swerveModules.size)
            for (i in states.indices) states[i] = swerveModules[i].measuredState
            return states
        }

    private val moduleLatestPositions: Array<SwerveModulePosition?>
        /**
         * Returns the module positions (turn angles and drive positions) for all the modules.
         */
        get() {
            val states = arrayOfNulls<SwerveModulePosition>(swerveModules.size)
            for (i in states.indices) states[i] = swerveModules[i].latestPosition
            return states
        }

    override fun getPose() = poseEstimator.estimatedPosition
    override fun setPose(pose: Pose2d) {
        poseEstimator.resetPosition(rawGyroRotation, moduleLatestPositions, pose)
    }

    override val rawGyroYaw: Rotation2d
        get() = gyroInputs.yawPosition

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

    init {
        this.gyroInputs = LoggedGyroIOInputs()
        this.rawGyroRotation = Rotation2d()
        this.swerveModules = arrayOf<SwerveModule>(
            SwerveModule(frontLeftModuleIO, "FrontLeft"),
            SwerveModule(frontRightModuleIO, "FrontRight"),
            SwerveModule(backLeftModuleIO, "BackLeft"),
            SwerveModule(backRightModuleIO, "BackRight"),
        )

        lastModulePositions = arrayOf(SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition())
        this.poseEstimator = SwerveDrivePoseEstimator(
            DRIVE_KINEMATICS, rawGyroRotation, lastModulePositions, Pose2d(),
            VecBuilder.fill(ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS, ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS, GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS),
            VecBuilder.fill(
                TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION,
                TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION,
                ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION
            )
        )

        this.odometryThread = OdometryThread.createInstance()
        this.odometryThreadInputs = LoggedOdometryThreadInputs()
        odometryThread.start()

        startDashboardDisplay()

        periodic()

        RobotPeriodicManager.startPeriodic {
            periodic()
            Logger.recordOutput("Odometry/Robot", getPose())
        }
    }

    private fun startDashboardDisplay() {
        SmartDashboard.putData("Swerve Drive") { builder: SendableBuilder ->
            builder.setSmartDashboardType("SwerveDrive")
            builder.addDoubleProperty("Front Left Angle", { swerveModules[0].steerFacing.radians }, null)
            builder.addDoubleProperty("Front Left Velocity", { swerveModules[0].driveVelocityMetersPerSec }, null)

            builder.addDoubleProperty("Front Right Angle", { swerveModules[0].steerFacing.radians }, null)
            builder.addDoubleProperty("Front Right Velocity", { swerveModules[0].driveVelocityMetersPerSec }, null)

            builder.addDoubleProperty("Back Left Angle", { swerveModules[0].steerFacing.radians }, null)
            builder.addDoubleProperty("Back Left Velocity", { swerveModules[0].driveVelocityMetersPerSec }, null)

            builder.addDoubleProperty("Back Right Angle", { swerveModules[0].steerFacing.radians }, null)
            builder.addDoubleProperty("Back Right Velocity", { swerveModules[0].driveVelocityMetersPerSec }, null)
            builder.addDoubleProperty("Robot Angle", { facing.minus(FieldConstants.driverStationFacing).radians }, null)
        }
    }
}