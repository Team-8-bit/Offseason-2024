package org.team9432.resources.swerve

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory
import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import kotlinx.coroutines.launch
import org.team9432.lib.LibraryState
import org.team9432.lib.coroutines.CoroutineRobot
import org.team9432.lib.coroutines.RobotScope
import org.team9432.lib.coroutines.robotPeriodic
import org.team9432.lib.doglog.Logger
import org.team9432.lib.resource.Action
import org.team9432.lib.resource.Resource
import org.team9432.lib.resource.toAction
import org.team9432.lib.resource.use
import org.team9432.oi.Buttons

object Swerve: Resource("Swerve") {
    private var hasAppliedOperatorPerspective = false

    private val swerve = TunerConstants.drivetrain

    private var currentState = swerve.state

    init {
        swerve.daqThread.setThreadPriority(99)

        if (LibraryState.isSimulation) {
            startSimThread()
        }

        RobotScope.launch {
            use(Swerve, name = "DefaultSetter") { } // make it so the default command runs, to be fixed later
        }

        CoroutineRobot.startPeriodic {
            currentState = swerve.state

            if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
                LibraryState.alliance?.let { allianceColor ->
                    swerve.setOperatorPerspectiveForward(getOperatorPerspective(allianceColor))
                    hasAppliedOperatorPerspective = true
                }
            }

            log()
        }
    }

    override val defaultAction: Action = {
        robotPeriodic {
            swerve.setControl(Buttons.getTeleopSwerveRequest())
            false
        }
    }

    private fun log() {
        Logger.log("Swerve/Pose", getRobotPose())
        Logger.log("Swerve/ModuleStates", currentState.ModuleStates)
        Logger.log("Swerve/ModuleTargets", currentState.ModuleTargets)
        Logger.log("Swerve/Speeds", getRobotSpeeds())
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

        Logger.log("Swerve/CurrentTrajectory", if (shouldMirrorTrajectory()) trajectory.flipped().poses else trajectory.poses)

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

    fun seedFieldRelative(pose: Pose2d) = swerve.seedFieldRelative(pose)
    fun seedFieldRelative() = swerve.seedFieldRelative()
    fun setVisionMeasurementStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) = swerve.setVisionMeasurementStdDevs(visionMeasurementStdDevs)
    fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double) = swerve.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds)

    fun getRobotPose(): Pose2d = currentState.Pose ?: Pose2d()
    fun getRobotTranslation(): Translation2d = getRobotPose().translation
    fun getRobotSpeeds(): ChassisSpeeds = currentState.speeds ?: ChassisSpeeds()

    fun getTalons(): List<TalonFX> = buildList {
        for (i in 0..3) {
            val module = swerve.getModule(i)
            add(module.driveMotor)
            add(module.steerMotor)
        }
    }
}