package org.team9432.resources.swerve

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import org.team9432.io.Buttons.controller
import org.team9432.lib.LibraryState
import org.team9432.lib.resource.Resource
import org.team9432.lib.robot.CoroutineRobot


object Swerve: Resource("Swerve") {
    val swerve = TunerConstants.drivetrain

    init {
        val request = SwerveRequest.FieldCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

        if (LibraryState.isSimulation) {
            startSimThread()
        }

        CoroutineRobot.addPeriodic {
            updateDrive(request)
            log()
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

    private fun updateDrive(request: SwerveRequest.FieldCentric) {
        swerve.setControl(
            request
                .withVelocityX(-controller.leftY * 5)
                .withVelocityY(-controller.leftX * 5)
                .withRotationalRate(-controller.rightX * Math.toRadians(360.0))
        )
    }

    private val publisher: StructPublisher<Pose2d> = table.getStructTopic("Pose", Pose2d.struct).publish()
    private val statePublisher: StructArrayPublisher<SwerveModuleState> = table.getStructArrayTopic("States", SwerveModuleState.struct).publish()
    private fun log() {
        publisher.set(swerve.state.Pose ?: Pose2d())
        statePublisher.set(swerve.state.ModuleStates)
    }
}