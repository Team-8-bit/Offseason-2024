package org.team9432.resources.swerve

import com.choreo.lib.ChoreoTrajectory
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.littletonrobotics.junction.Logger
import org.team9432.Robot
import org.team9432.RobotSim
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.simulation.competitionfield.simulations.SwerveDriveSimulation
import org.team9432.lib.util.*
import org.team9432.resources.swerve.DrivetrainConstants.CHASSIS_MAX_VELOCITY
import org.team9432.resources.swerve.DrivetrainConstants.DRIVE_KINEMATICS
import org.team9432.resources.swerve.DrivetrainConstants.MODULE_TRANSLATIONS
import org.team9432.resources.swerve.DrivetrainConstants.simProfile
import org.team9432.resources.swerve.Swerve.runRawChassisSpeeds
import org.team9432.resources.swerve.gyro.GyroIO
import org.team9432.resources.swerve.gyro.GyroIOPigeon2
import org.team9432.resources.swerve.gyro.GyroIOSim
import org.team9432.resources.swerve.gyro.LoggedGyroIOInputs
import org.team9432.resources.swerve.module.ModuleIOKraken
import org.team9432.resources.swerve.module.ModuleIOSim
import org.team9432.resources.swerve.module.SwerveModule
import org.team9432.resources.swerve.odometrythread.LoggedOdometryThreadInputs
import org.team9432.resources.swerve.odometrythread.OdometryThread
import java.util.concurrent.locks.ReentrantLock
import kotlin.math.abs
import kotlin.math.max
import kotlin.properties.Delegates

object Swerve {
    private val gyroInputs = LoggedGyroIOInputs()
    private val odometryThreadInputs = LoggedOdometryThreadInputs()

    private val gyroIO: GyroIO
    private val modules: Array<SwerveModule>

    private var rawGyroRotation: Rotation2d = Rotation2d()
    private val lastModulePositions: Array<SwerveModulePosition> = Array(4) { SwerveModulePosition() }

    private val poseEstimator: SwerveDrivePoseEstimator = SwerveDrivePoseEstimator(DRIVE_KINEMATICS, rawGyroRotation, lastModulePositions, Pose2d())

    private val odometryThread = OdometryThread.createInstance()
    val odometryLock = ReentrantLock()

    var robotSimulation: RobotSim by Delegates.notNull() // This is only initialized when the robot is simulated
        private set

    private var swerveSim: SwerveDriveSimulation by Delegates.notNull() // This is only initialized when the robot is simulated

    init {
        val frontLeft = simSwitch(real = ModuleIOKraken(TunerConstants.FrontLeft, TunerConstants.kCANbusName), sim = ModuleIOSim())
        val frontRight = simSwitch(real = ModuleIOKraken(TunerConstants.FrontRight, TunerConstants.kCANbusName), sim = ModuleIOSim())
        val backLeft = simSwitch(real = ModuleIOKraken(TunerConstants.BackLeft, TunerConstants.kCANbusName), sim = ModuleIOSim())
        val backRight = simSwitch(real = ModuleIOKraken(TunerConstants.BackRight, TunerConstants.kCANbusName), sim = ModuleIOSim())
        gyroIO = simSwitch(real = GyroIOPigeon2(), sim = GyroIOSim())

        modules = arrayOf(
            SwerveModule(frontLeft, "FrontLeft"),
            SwerveModule(frontRight, "FrontRight"),
            SwerveModule(backLeft, "BackLeft"),
            SwerveModule(backRight, "BackRight"),
        )

        if (Robot.isSimulated) {
            swerveSim = SwerveDriveSimulation(
                simProfile,
                gyroIO as GyroIOSim,
                frontLeft as ModuleIOSim,
                frontRight as ModuleIOSim,
                backLeft as ModuleIOSim,
                backRight as ModuleIOSim,
                startingPose = Pose2d(3.0, 2.0, Rotation2d()).applyFlip(),
                ::resetOdometry
            )

            robotSimulation = RobotSim(swerveSim)
        }

        odometryThread.start()

        periodic()

        RobotPeriodicManager.startPeriodic {
            periodic()
            Logger.recordOutput("Odometry/Robot", this@Swerve.getRobotPose())
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
        Logger.recordOutput("Drive/OdometryPose", this.getRobotPose())
        Logger.recordOutput("Drive/MeasuredModuleStates", *moduleStates)

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

    private val xPid = PIDController(5.0, 0.0, 0.0)
    private val yPid = PIDController(5.0, 0.0, 0.0)
    private val rPid = PIDController(5.0, 0.0, 0.5)

    suspend fun followChoreo(trajectory: ChoreoTrajectory) {
        val controlFunction = ChoreoUtil.choreoSwerveController(xPid, yPid, rPid, ::getRobotPose)

        Logger.recordOutput("Swerve/CurrentTrajectory", *allianceSwitch(blue = trajectory.poses, red = trajectory.flipped().poses))

        ChoreoUtil.choreoSwerveAction(trajectory, controlFunction) { speedsToApply ->
            runRawChassisSpeeds(speedsToApply)
        }
    }

    val rawGyroYaw: Rotation2d
        get() = gyroInputs.yawPosition

    private var lastSetpoints = emptyArray<SwerveModuleState>()
    fun runRawChassisSpeeds(speeds: ChassisSpeeds) {
        val setpointStates: Array<SwerveModuleState> = DRIVE_KINEMATICS.toSwerveModuleStates(speeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, CHASSIS_MAX_VELOCITY)

        // Send setpoints to modules
        val optimizedSetpointStates = if (abs(speeds.vxMetersPerSecond) < 0.001 && abs(speeds.vyMetersPerSecond) < 0.001 && abs(speeds.omegaRadiansPerSecond) < 0.001) {
            lastSetpoints.mapIndexed { index, swerveModuleState -> modules[index].runSetpoint(SwerveModuleState(0.0, swerveModuleState.angle)) }.toTypedArray()
        } else {
            Array(4) { index -> modules[index].runSetpoint(setpointStates[index]) }
        }

        lastSetpoints = optimizedSetpointStates

        Logger.recordOutput("Drive/Setpoints", *setpointStates)
        Logger.recordOutput("Drive/SetpointsOptimized", *optimizedSetpointStates)
    }

    fun runFieldCentricChassisSpeeds(fieldCentricSpeeds: ChassisSpeeds) {
        runRawChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(fieldCentricSpeeds, this.getRobotPose().rotation))
    }

    /**
     * Locks the chassis and turns the modules to an X formation to resist movement.
     * The lock will be cancelled the next time a nonzero velocity is requested.
     */
    fun lockChassisWithXFormation() {
        val swerveHeadings = Array<Rotation2d>(modules.size) { index -> MODULE_TRANSLATIONS[index].angle }
        DRIVE_KINEMATICS.resetHeadings(*swerveHeadings)
    }

    /** Returns the module states (turn angles and drive velocities) for all the modules. */
    private val moduleStates: Array<SwerveModuleState>
        get() = Array(modules.size) { index -> modules[index].measuredState }

    /** Returns the module positions (turn angles and drive positions) for all the modules. */
    private val moduleLatestPositions: Array<SwerveModulePosition?>
        get() = Array(modules.size) { index -> modules[index].latestPosition }

    fun getRobotPose(): Pose2d = poseEstimator.estimatedPosition
    fun resetOdometry(pose: Pose2d) = poseEstimator.resetPosition(pose.rotation, moduleLatestPositions, pose)
    fun setActualSimPose(pose: Pose2d) = swerveSim.setSimulationWorldPose(pose)

    fun getRobotTranslation(): Translation2d = Swerve.getRobotPose().translation
    fun getRobotRelativeSpeeds(): ChassisSpeeds = DRIVE_KINEMATICS.toChassisSpeeds(*moduleStates)

    fun addVisionMeasurement(visionPose: Pose2d, timestamp: Double, measurementStdDevs: Matrix<N3, N1>) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp, measurementStdDevs)
        previousVisionMeasurementTimeStamp = max(timestamp, previousVisionMeasurementTimeStamp)
    }

    fun setGyroAngle(angle: Rotation2d) = gyroIO.setAngle(angle)

    private var previousVisionMeasurementTimeStamp: Double = -1.0
}