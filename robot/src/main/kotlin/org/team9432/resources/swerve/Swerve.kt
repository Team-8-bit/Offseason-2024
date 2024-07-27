package org.team9432.resources.swerve

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import kotlinx.coroutines.launch
import org.team9432.PositionConstants
import org.team9432.io.Buttons
import org.team9432.lib.LibraryState
import org.team9432.lib.coroutines.robotPeriodic
import org.team9432.lib.resource.Action
import org.team9432.lib.resource.Resource
import org.team9432.lib.resource.toAction
import org.team9432.lib.resource.use
import org.team9432.lib.robot.CoroutineRobot
import org.team9432.lib.robot.RobotScope
import org.team9432.lib.unit.Length
import org.team9432.lib.unit.inMeters
import org.team9432.lib.unit.meters
import org.team9432.lib.util.PoseUtil.applyFlip
import org.team9432.lib.util.set
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot

object Swerve: Resource("Swerve") {
    private var hasAppliedOperatorPerspective = false

    val swerve = TunerConstants.drivetrain
    var currentState = SwerveDriveState()
        private set

    init {
        swerve.daqThread.setThreadPriority(99)

        if (LibraryState.isSimulation) {
            startSimThread()
        }

        RobotScope.launch {
            use(Swerve) { } // make it so the default command runs, to be fixed later
        }

        CoroutineRobot.startPeriodic {
            currentState = swerve.getState()

            log()

            if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
                LibraryState.alliance?.let { allianceColor ->
                    swerve.setOperatorPerspectiveForward(getOperatorPerspective(allianceColor))
                    hasAppliedOperatorPerspective = true
                }
            }
        }
    }

    override val defaultAction: Action = {
        robotPeriodic {
            swerve.setControl(Buttons.getTeleopSwerveRequest())
            false
        }
    }

    private val publisher: StructPublisher<Pose2d> = table.getStructTopic("Pose", Pose2d.struct).publish()
    private val statePublisher: StructArrayPublisher<SwerveModuleState> = table.getStructArrayTopic("States", SwerveModuleState.struct).publish()
    private val trajPublisher: StructArrayPublisher<Pose2d> = table.getStructArrayTopic("Traj", Pose2d.struct).publish()

    private fun log() {
        publisher.set(currentState.Pose ?: Pose2d())
        statePublisher.set(currentState.ModuleStates)
        table.set("SpeakerDistance", distanceToSpeaker().inMeters)
    }

    private val xPid = PIDController(1.0, 0.0, 0.0)
    private val yPid = PIDController(1.0, 0.0, 0.0)
    private val rPid = PIDController(1.0, 0.0, 0.0)

    suspend fun followChoreo(trajectory: ChoreoTrajectory) {
        val poseSupplier = { swerve.state.Pose }
        val choreoControlFunction = Choreo.choreoSwerveController(xPid, yPid, rPid)
        val speedsRequest = SwerveRequest.ApplyChassisSpeeds()
        val outputChassisSpeeds: (ChassisSpeeds) -> Unit = { swerve.setControl(speedsRequest.withSpeeds(it)) }
        val shouldMirrorTrajectory: () -> Boolean = { LibraryState.alliance == Alliance.Red }

        if (shouldMirrorTrajectory()) trajPublisher.set(trajectory.flipped().poses)
        else trajPublisher.set(trajectory.poses)

        val command = Choreo.choreoSwerveCommand(trajectory, poseSupplier, choreoControlFunction, outputChassisSpeeds, shouldMirrorTrajectory)

        use(Swerve, action = command.toAction())
    }

    private fun getOperatorPerspective(alliance: Alliance): Rotation2d {
        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        val bluePerspectiveRotation = Rotation2d.fromDegrees(0.0)

        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        val redPerspectiveRotation = Rotation2d.fromDegrees(180.0)

        return when (alliance) {
            Alliance.Red -> redPerspectiveRotation
            Alliance.Blue -> bluePerspectiveRotation
        }
    }

    private const val SIM_LOOP_PERIOD: Double = 0.005 // 5 ms
    private var simNotifier: Notifier? = null
    private var lastSimTime = 0.0

    private fun startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds()

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = Notifier {
            val currentTime = Utils.getCurrentTimeSeconds()
            val deltaTime = currentTime - lastSimTime
            lastSimTime = currentTime

            /* use the measured time delta, get battery voltage from WPILib */
            swerve.updateSimState(deltaTime, RobotController.getBatteryVoltage())
        }.also {
            it.startPeriodic(SIM_LOOP_PERIOD)
        }
    }

    fun getPose() = currentState.Pose

    /** Returns the angle between the two given positions. */
    fun angleTo(pose: Translation2d, currentPose: Translation2d = getPose().translation): Rotation2d {
        return Rotation2d(atan2(pose.y - currentPose.y, pose.x - currentPose.x))
    }

    /** Returns true if the robot is within [epsilon] of the given pose. */
    fun isNear(pose: Pose2d, epsilon: Length): Boolean {
        val robotPose = getPose()
        return hypot(robotPose.x - pose.x, robotPose.y - pose.y) < epsilon.inMeters
    }

    /** Return the distance from the robot to another point. */
    fun distanceTo(pose: Translation2d): Length {
        val robotPose = getPose()
        return robotPose.translation.getDistance(pose).meters
    }

    /** Return the distance from the robot to the speaker. */
    fun distanceToSpeaker() = distanceTo(PositionConstants.speakerAimPose.applyFlip())

    /** Returns true if this double is within [range] of [other]. */
    fun Double.isCloseTo(other: Double, range: Double) = abs(this - other) < range
}