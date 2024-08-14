package org.team9432.oi

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import org.team9432.Actions
import org.team9432.RobotController
import org.team9432.lib.input.XboxController
import org.team9432.resources.swerve.Swerve
import kotlin.math.pow
import kotlin.math.withSign

object Controls {
    val controller = XboxController(0)

    private val teleopRequest: SwerveRequest.FieldCentric = SwerveRequest.FieldCentric()
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

    fun getTeleopSwerveRequest(): SwerveRequest.FieldCentric =
        teleopRequest
            .withVelocityX(getTranslationalSpeed(-controller.leftYRaw))
            .withVelocityY(getTranslationalSpeed(-controller.leftXRaw))
            .withRotationalRate(getRotationalSpeed())

    init {
        controller.y.onTrue { RobotController.setAction { Actions.idle() } }

        controller.leftBumper
            .onTrue { RobotController.setAction { Actions.runIntake() } }

        controller.b
            .onTrue { RobotController.setAction { Actions.visionShoot() } }

        controller.a
            .onTrue { RobotController.setAction { Actions.amp() } }

        controller.back
            .onTrue { Swerve.seedFieldRelative() }

        controller.start
            .onTrue { RobotController.setAction { Actions.outtake() } }
            .onFalse { RobotController.setAction { Actions.idle() } }
    }


    private fun getRotationalSpeed(): Double {
        return getTriggerRotationSpeed() + getJoystickRotationSpeed()
    }

    private fun getJoystickRotationSpeed(): Double {
        return -controller.rightX * Math.toRadians(360.0)
    }

    private fun getTriggerRotationSpeed(): Double {
        val rightAxis = controller.rightTriggerAxis
        val leftAxis = controller.leftTriggerAxis
        return ((rightAxis.pow(2) * -1) + leftAxis.pow(2)) * Math.toRadians(270.0)
    }

    private fun getTranslationalSpeed(rawJoystick: Double): Double {
        val speedSquared = (rawJoystick * rawJoystick).withSign(rawJoystick)
        return speedSquared * if (controller.rightBumper.invoke()) 3.0 else 5.0
    }
}