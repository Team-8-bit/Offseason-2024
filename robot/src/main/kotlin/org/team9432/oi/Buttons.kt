package org.team9432.oi

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import org.team9432.Actions
import org.team9432.lib.input.XboxController
import org.team9432.lib.resource.use
import org.team9432.resources.Intake
import org.team9432.resources.Loader
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.math.pow
import kotlin.math.withSign

object Buttons {
    private val controller = XboxController(0)

    private val teleopRequest: SwerveRequest.FieldCentric = SwerveRequest.FieldCentric()
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

    fun getTeleopSwerveRequest(): SwerveRequest.FieldCentric =
        teleopRequest
            .withVelocityX(getTranslationalSpeed(-controller.leftYRaw))
            .withVelocityY(getTranslationalSpeed(-controller.leftXRaw))
            .withRotationalRate(getRotationalSpeed())

    init {
        controller.y.onTrue {
            use(Intake, Shooter, Loader, Swerve) {
                Intake.setState(Intake.State.IDLE)
                Shooter.setState(Shooter.State.IDLE)
                Loader.setState(Loader.State.IDLE)
            }
        }

        controller.leftBumper
            .onTrue { Actions.startIntaking() }
            .onFalse { Actions.stopIntaking() }

        controller.b
            .whileTrue { Actions.pullNoteAndSpinUpTo(Shooter.State.VISION_SHOOT) }
            .onFalse { Actions.shootAndSpinDown() }

        controller.x
            .whileTrue { Actions.pullNoteAndSpinUpTo(Shooter.State.SUBWOOFER) }
            .onFalse { Actions.shootAndSpinDown() }

        controller.a
            .whileTrue { Actions.pullNoteAndSpinUpTo(Shooter.State.AMP) }
            .onFalse { Actions.shootAndSpinDown() }

        controller.back
            .onTrue { Swerve.seedFieldRelative() }
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