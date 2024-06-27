package org.team9432.resources.swerve

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import org.team9432.io.Buttons.controller
import org.team9432.lib.resource.Resource
import org.team9432.lib.robot.CoroutineRobot


object Swerve: Resource("Swerve") {
    val swerve = TunerConstants.drivetrain

    init {
        val request = SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

        CoroutineRobot.addPeriodic {
            swerve.setControl(
                request
                    .withVelocityX(-controller.leftX)
                    .withVelocityY(-controller.leftY)
                    .withRotationalRate(-controller.rightX)
            )
        }
    }
}