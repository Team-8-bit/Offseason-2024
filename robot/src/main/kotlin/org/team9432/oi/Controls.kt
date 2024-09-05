package org.team9432.oi

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import org.team9432.*
import org.team9432.lib.input.XboxController
import org.team9432.lib.unit.asRotation2d
import org.team9432.lib.unit.degrees
import org.team9432.lib.unit.meters
import org.team9432.lib.util.allianceSwitch
import org.team9432.lib.util.angleTo
import org.team9432.resources.intake.Intake
import org.team9432.resources.loader.Loader
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.math.hypot
import kotlin.math.pow


object Controls {
    val controller = XboxController(0)

    var forceDisableVision = false
        private set

    private val ratelimitX = SlewRateLimiter(20.0)
    private val ratelimitY = SlewRateLimiter(20.0)

    private val teleopRequest: SwerveRequest.FieldCentric = SwerveRequest.FieldCentric()
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)

    private val teleAimRequest: SwerveRequest.FieldCentricFacingAngle = SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
        .apply {
            HeadingController.enableContinuousInput(-Math.PI, Math.PI)
            HeadingController.p = 5.0
        }

    private val shouldAimAtSpeaker
        get() =
            getRotationalSpeed() == 0.0 &&
                    Shooter.isShootingSpeaker &&
                    Shooter.distanceToSpeaker() < 5.0.meters &&
                    Beambreaks.hasNote &&
                    Vision.isEnabled

    private val shouldAimAtAmp
        get() =
            getRotationalSpeed() == 0.0 &&
                    Shooter.isShootingAmp &&
                    Beambreaks.hasNote &&
                    Vision.isEnabled

    fun getTeleopSwerveRequest(): SwerveRequest {
        return when {
            shouldAimAtSpeaker -> teleAimRequest.apply {
                val speed = getTranslationalSpeed()
                withVelocityX(ratelimitX.calculate(speed.x * 5.0))
                withVelocityY(ratelimitY.calculate(speed.y * 5.0))
                withTargetDirection(Swerve.getRobotTranslation().angleTo(PositionConstants.speakerAimPose).asRotation2d.let { allianceSwitch(blue = it, red = it.plus(Rotation2d.fromDegrees(180.0))) })
            }

            shouldAimAtAmp -> teleAimRequest.apply {
                val speed = getTranslationalSpeed()
                withVelocityX(ratelimitX.calculate(speed.x * 5.0))
                withVelocityY(ratelimitY.calculate(speed.y * 5.0))
                withTargetDirection(90.degrees.asRotation2d.let { allianceSwitch(blue = it, red = it.plus(Rotation2d.fromDegrees(180.0))) })
            }

            else -> teleopRequest.apply {
                val speed = getTranslationalSpeed()
                withVelocityX(ratelimitX.calculate(speed.x * 5.0))
                withVelocityY(ratelimitY.calculate(speed.y * 5.0))
                withRotationalRate(getRotationalSpeed())
            }
        }
    }

    init {
        controller.y.onTrue { RobotController.setAction { Actions.idle() } }

        controller.x.onTrue { forceDisableVision = true }
        controller.a.onTrue { forceDisableVision = false }

        controller.leftBumper.and { Shooter.isIdle }
            .onTrue { RobotController.setAction { Actions.intake() } }

        controller.rightBumper.and { Beambreaks.hasNote && Intake.isIdle && Loader.isIdle }
            .onTrue {
                RobotController.setAction {
                    if (Vision.isEnabled) {
                        Actions.visionShoot()
                    } else {
                        Actions.dashboardShoot()
                    }
                }
            }

        controller.b
            .onTrue { RobotController.setAction { Actions.amp() } }

        controller.back
            .onTrue { Swerve.seedFieldRelative() }

        controller.start
            .onTrue { RobotController.setAction { Actions.outtake() } }
            .onFalse { RobotController.setAction { Actions.idle() } }

        controller.povRight.onTrue { RobotController.setAction { Actions.feedNote() } }
    }


    private fun getRotationalSpeed(): Double {
        return getTriggerRotationSpeed() //+ getJoystickRotationSpeed()
    }

    private fun getJoystickRotationSpeed(): Double {
        return -controller.rightX * Math.toRadians(360.0)
    }

    private fun getTriggerRotationSpeed(): Double {
        val rightAxis = controller.rightTriggerAxis
        val leftAxis = controller.leftTriggerAxis
        return ((rightAxis.pow(2) * -1) + leftAxis.pow(2)) * Math.toRadians(360.0)
    }

    // https://github.com/Mechanical-Advantage/RobotCode2024/blob/a025615a52193b7709db7cf14c51c57be17826f2/src/main/java/org/littletonrobotics/frc2024/subsystems/drive/controllers/TeleopDriveController.java#L83
    private fun getTranslationalSpeed(): Translation2d {
        val x: Double = -controller.leftYRaw
        val y: Double = -controller.leftXRaw

        val deadband = 0.15

        // Apply deadband
        var linearMagnitude = MathUtil.applyDeadband(hypot(x, y), deadband)
        val linearDirection = Rotation2d(x, y)

        // Square magnitude
        linearMagnitude = linearMagnitude * linearMagnitude

        // Calcaulate new linear velocity
        val linearVelocity =
            Pose2d(Translation2d(), linearDirection)
                .transformBy(Transform2d(linearMagnitude, 0.0, Rotation2d()))
                .translation

        return linearVelocity
    }
}