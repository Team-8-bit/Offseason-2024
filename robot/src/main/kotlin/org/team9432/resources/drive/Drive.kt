package org.team9432.resources.drive

import com.choreo.lib.ChoreoTrajectory
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team9432.Robot
import org.team9432.RobotPosition
import org.team9432.lib.simulation.competitionfield.simulations.SwerveDriveSimulation
import org.team9432.lib.util.SwerveSetpointGenerator
import org.team9432.lib.util.SwerveUtil
import org.team9432.resources.drive.DrivetrainConstants.DRIVE_KINEMATICS
import org.team9432.resources.drive.controllers.ChoreoTrajectoryController
import org.team9432.resources.drive.controllers.TeleopAutoAimController
import org.team9432.resources.drive.controllers.TeleopDriveController
import org.team9432.resources.drive.gyro.GyroIO
import org.team9432.resources.drive.gyro.LoggedGyroIOInputs
import org.team9432.resources.drive.module.ModuleIO
import org.team9432.resources.drive.module.SwerveModule
import org.team9432.resources.drive.odometrythread.LoggedOdometryThreadInputs
import org.team9432.resources.drive.odometrythread.OdometryThread
import java.util.concurrent.locks.ReentrantLock
import kotlin.concurrent.withLock
import kotlin.properties.Delegates

class Drive(
    private val gyroIO: GyroIO,
    frontLeft: ModuleIO,
    frontRight: ModuleIO,
    backLeft: ModuleIO,
    backRight: ModuleIO,
): SubsystemBase() {
    enum class ControlMode {
        TELEOP,
        TRAJECTORY,
        WHEEL_RADIUS_CHARACTERIZATION
    }

    private val gyroInputs = LoggedGyroIOInputs()
    private val odometryThreadInputs = LoggedOdometryThreadInputs()

    private val modules: Array<SwerveModule> = arrayOf(
        SwerveModule(frontLeft, "FrontLeft"),
        SwerveModule(frontRight, "FrontRight"),
        SwerveModule(backLeft, "BackLeft"),
        SwerveModule(backRight, "BackRight"),
    )

    private var rawGyroRotation: Rotation2d = Rotation2d()
    private val lastModulePositions: Array<SwerveModulePosition> = Array(4) { SwerveModulePosition() }

    private val odometryThread = OdometryThread.createInstance()

    private var currentControlMode = ControlMode.TELEOP

    private var characterizationInput = 0.0
    private var teleopAutoAimController: TeleopAutoAimController? = null
    private var trajectoryController: ChoreoTrajectoryController? = null
    private var teleopDriveController = TeleopDriveController()

    private val setpointGenerator = SwerveSetpointGenerator(DRIVE_KINEMATICS, DrivetrainConstants.MODULE_TRANSLATIONS.toTypedArray())
    private var currentSetpoint = SwerveSetpointGenerator.SwerveSetpoint(ChassisSpeeds(), Array(4) { SwerveModuleState() })

    private val swerveLimits = SwerveSetpointGenerator.ModuleLimits(
        maxDriveVelocity = 4.0,
        maxDriveAcceleration = 20.0,
        maxSteeringVelocity = Units.degreesToRadians(1080.0)
    )

    private var desiredChassisSpeeds = ChassisSpeeds()

    companion object {
        val odometryLock = ReentrantLock()
    }

    private var swerveSim: SwerveDriveSimulation by Delegates.notNull() // This is only initialized when the robot is simulated

    init {
        odometryThread.start()
    }

    override fun periodic() {
        updateOdometry()
        updateControl()

        currentSetpoint = setpointGenerator.generateSetpoint(
            limits = swerveLimits,
            prevSetpoint = currentSetpoint,
            desiredState = desiredChassisSpeeds,
            dt = Robot.periodSeconds
        )

        for (i in modules.indices) {
            modules[i].runSetpoint(currentSetpoint.moduleStates[i])
        }

        Logger.recordOutput("Drive/Un254dSetpoints", *DRIVE_KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds))
        Logger.recordOutput("Drive/Setpoints", *currentSetpoint.moduleStates)
        Logger.recordOutput("Drive/DesiredSpeeds", desiredChassisSpeeds)
        Logger.recordOutput("Drive/SetpointSpeeds", currentSetpoint.chassisSpeeds)
        Logger.recordOutput("Drive/ControlMode", currentControlMode)
    }

    private fun updateControl() {
        val teleopSpeeds = teleopDriveController.calculate()
        when (currentControlMode) {
            ControlMode.TELEOP -> {
                desiredChassisSpeeds = teleopSpeeds

                val autoAimRadPerSec = teleopAutoAimController?.calculate()
                if (autoAimRadPerSec != null) {
                    desiredChassisSpeeds.omegaRadiansPerSecond = autoAimRadPerSec
                }
            }

            ControlMode.TRAJECTORY -> {
                desiredChassisSpeeds = trajectoryController?.calculate() ?: ChassisSpeeds()
            }

            ControlMode.WHEEL_RADIUS_CHARACTERIZATION -> {
                desiredChassisSpeeds = ChassisSpeeds(0.0, 0.0, characterizationInput)
            }
        }
    }

    private fun updateOdometry() {
        odometryLock.withLock {
            gyroIO.updateInputs(gyroInputs)
            Logger.processInputs("Drive/Gyro", gyroInputs)

            odometryThread.updateInputs(odometryThreadInputs)
            Logger.processInputs("Drive/OdometryThread", odometryThreadInputs)

            modules.forEach(SwerveModule::updateInputs)
        }

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

            RobotPosition.applyOdometryObservation(timestampSamples[timestampIndex], rawGyroRotation, modulePositions)

            val robotRelativeSpeeds = DRIVE_KINEMATICS.toChassisSpeeds(*moduleStates)
            if (gyroInputs.connected) {
                robotRelativeSpeeds.omegaRadiansPerSecond = gyroInputs.yawVelocityRadPerSec
            }
            RobotPosition.addVelocityData(robotRelativeSpeeds)
        }
    }

    fun acceptTeleopInput(xInput: Double, yInput: Double, rInput: Double) {
        if (DriverStation.isTeleopEnabled()) {
            currentControlMode = ControlMode.TELEOP
            teleopDriveController.acceptControllerInput(xInput, yInput, rInput)
        }
    }

    fun setTrajectory(trajectory: ChoreoTrajectory) {
        if (DriverStation.isAutonomousEnabled()) {
            currentControlMode = ControlMode.TRAJECTORY
            trajectoryController = ChoreoTrajectoryController(trajectory)
        }
    }

    fun clearTrajectory() {
        trajectoryController = null
        currentControlMode = ControlMode.TELEOP
    }

    fun setAutoAimGoal(goal: () -> Rotation2d) {
        teleopAutoAimController = TeleopAutoAimController(goal)
    }

    fun atAutoAimGoal() = teleopAutoAimController == null || teleopAutoAimController?.atGoal() == true

    fun clearAutoAimGoal() {
        teleopAutoAimController = null
    }

    fun runWheelRadiusCharacterization(headingRadPerSec: Double) {
        currentControlMode = ControlMode.WHEEL_RADIUS_CHARACTERIZATION
        characterizationInput = headingRadPerSec
    }

    fun endCharacterization() {
        currentControlMode = ControlMode.TELEOP
    }

    fun getWheelRadiusCharacterizationPositions(): List<Double> {
        return modules.map { it.drivePositionRads }
    }

    private val xChoreoPid = PIDController(1.0, 0.0, 0.0)
    private val yChoreoPid = PIDController(1.0, 0.0, 0.0)
    private val rChoreoPid = PIDController(1.0, 0.0, 0.0)

//    suspend fun followChoreo(trajectory: ChoreoTrajectory) {
//        val controlFunction = ChoreoUtil.choreoSwerveController(xChoreoPid, yChoreoPid, rChoreoPid, ::getRobotPose)
//
//        Logger.recordOutput("Drive/CurrentTrajectory", *allianceSwitch(blue = trajectory.poses, red = trajectory.flipped().poses))
//        Logger.recordOutput("Drive/TrajectoryEndPose", trajectory.finalPose.applyFlip())
//
//        ChoreoUtil.choreoSwerveAction(trajectory, controlFunction) { speedsToApply ->
//            runRawChassisSpeeds(speedsToApply)
//        }
//        driveTo(trajectory.finalPose.applyFlip(), timeout = 1.0)
//        runRawChassisSpeeds(ChassisSpeeds())
//    }
//
//    private val xPid = PIDController(5.0, 0.0, 0.0)
//    private val yPid = PIDController(5.0, 0.0, 0.0)
//    private val rPid = PIDController(5.0, 0.0, 0.5).apply {
//        enableContinuousInput(-Math.PI, Math.PI)
//    }
//
//    suspend fun driveTo(targetPosition: Pose2d, timeout: Double? = null) = suspendCancellableCoroutine { cont ->
//        val timer = if (timeout != null) Timer() else null
//        timer?.restart()
//
//        val periodic = RobotPeriodicManager.startPeriodic {
//            val pose = getRobotPose()
//
//            val xFeedback = xPid.calculate(pose.x, targetPosition.x)
//            val yFeedback = yPid.calculate(pose.y, targetPosition.y)
//            val rotationFeedback = rPid.calculate(pose.rotation.radians, targetPosition.rotation.radians)
//
//            runFieldRelativeChassisSpeeds(
//                ChassisSpeeds(
//                    xFeedback + pose.x - targetPosition.x,
//                    yFeedback + pose.y - targetPosition.y,
//                    rotationFeedback
//                )
//            )
//
//            if (pose.distanceTo(targetPosition) < 0.05.meters && abs(pose.rotation.degrees - targetPosition.rotation.degrees) < 2
//                || timeout?.let { timer?.hasElapsed(it) } == true) {
//                timer?.stop()
//                runRawChassisSpeeds(ChassisSpeeds())
//
//                this.stopPeriodic()
//                cont.resume(Unit)
//            }
//        }
//
//        cont.invokeOnCancellation {
//            timer?.stop()
//            runRawChassisSpeeds(ChassisSpeeds())
//            periodic.stopPeriodic()
//        }
//    }

    /** Returns the module states (turn angles and drive velocities) for all the modules. */
    private val moduleStates: Array<SwerveModuleState>
        get() = Array(modules.size) { index -> modules[index].measuredState }

    /** Returns the module positions (turn angles and drive positions) for all the modules. */
    private val moduleLatestPositions: Array<SwerveModulePosition?>
        get() = Array(modules.size) { index -> modules[index].latestPosition }

    //    fun getRobotPose(): Pose2d = poseEstimator.estimatedPosition
//    fun resetOdometry(pose: Pose2d) = poseEstimator.resetPosition(pose.rotation, moduleLatestPositions, pose)
    fun setActualSimPose(pose: Pose2d) = swerveSim.setSimulationWorldPose(pose)

    fun setGyroAngle(angle: Rotation2d) = gyroIO.setAngle(angle)
}