package org.team9432.resources.swerve

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.networktables.StructPublisher
import org.team9432.io.Buttons.controller
import org.team9432.lib.resource.Resource
import org.team9432.lib.robot.CoroutineRobot


object Swerve: Resource("Swerve") {
    val swerve = TunerConstants.drivetrain

    init {
        val request =
            SwerveRequest.RobotCentric()
//            SwerveRequest.FieldCentric()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

        CoroutineRobot.addPeriodic {
            updateDrive(request)
            log()
        }
    }

    private fun updateDrive(request: SwerveRequest.RobotCentric) {
        swerve.setControl(
            request
                .withVelocityX(-controller.leftY * 2)
                .withVelocityY(-controller.leftX * 2)
                .withRotationalRate(-controller.rightX * 2)
        )
    }

    private val publisher: StructPublisher<Pose2d> = table.getStructTopic("Pose", Pose2d.struct).publish()
    private val statePublisher: StructArrayPublisher<SwerveModuleState> = table.getStructArrayTopic("States", SwerveModuleState.struct).publish()
    private fun log() {
        publisher.set(swerve.state.Pose ?: Pose2d())
        statePublisher.set(swerve.state.ModuleStates)
    }
}