package org.team9432.oi

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.team9432.*
import org.team9432.lib.input.XboxController
import org.team9432.lib.unit.asRotation2d
import org.team9432.lib.unit.degrees
import org.team9432.lib.unit.meters
import org.team9432.lib.util.allianceSwitch
import org.team9432.lib.util.angleTo
import org.team9432.lib.util.velocityLessThan
import org.team9432.resources.intake.Intake
import org.team9432.resources.loader.Loader
import org.team9432.resources.shooter.Shooter
import org.team9432.resources.swerve.DrivetrainConstants
import org.team9432.resources.swerve.Swerve
import org.team9432.vision.Vision
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.withSign


object Controls {
    val controller = XboxController(0)

    var forceDisableVision = false
        private set

    private val ratelimitX = SlewRateLimiter(20.0)
    private val ratelimitY = SlewRateLimiter(20.0)

    private val headingPID = PIDController(5.0, 0.0, 0.5).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }

    private val shouldAimAtSpeaker
        get() =
            getRotationalSpeed() == 0.0 &&
                    Swerve.getRobotRelativeSpeeds().velocityLessThan(metersPerSecond = 1.0, rotationsPerSecond = Double.MAX_VALUE) && // Don't aim if the robot is driving fast
                    Shooter.isShootingSpeaker &&
                    Shooter.distanceToSpeaker() < 3.0.meters &&
                    Beambreaks.hasNote &&
                    Vision.isEnabled

    private val shouldAimAtAmp
        get() =
            getRotationalSpeed() == 0.0 &&
                    Shooter.isShootingAmp &&
                    Beambreaks.hasNote &&
                    Vision.isEnabled

    private var wasJustUsingHeadingController = false

    fun getTeleopSwerveRequest(): ChassisSpeeds {
        return ChassisSpeeds.discretize(when {
            shouldAimAtSpeaker -> ChassisSpeeds().apply {
                if (!wasJustUsingHeadingController) headingPID.reset()
                val speed = getTranslationalSpeed()
                vxMetersPerSecond = ratelimitX.calculate(speed.x * allianceSwitch(blue = 1, red = -1))
                vyMetersPerSecond = ratelimitY.calculate(speed.y * allianceSwitch(blue = 1, red = -1))
                omegaRadiansPerSecond = headingPID.calculate(Swerve.getRobotPose().rotation.radians, Swerve.getRobotTranslation().angleTo(PositionConstants.speakerAimPose).asRotation2d.radians)
                wasJustUsingHeadingController = true
            }

            shouldAimAtAmp -> ChassisSpeeds().apply {
                if (!wasJustUsingHeadingController) headingPID.reset()
                val speed = getTranslationalSpeed()
                vxMetersPerSecond = ratelimitX.calculate(speed.x * allianceSwitch(blue = 1, red = -1))
                vyMetersPerSecond = ratelimitY.calculate(speed.y * allianceSwitch(blue = 1, red = -1))
                omegaRadiansPerSecond = headingPID.calculate(Swerve.getRobotPose().rotation.radians, 90.degrees.asRotation2d.radians)
                wasJustUsingHeadingController = true
            }

            else -> ChassisSpeeds().apply {
                val speed = getTranslationalSpeed()
                vxMetersPerSecond = ratelimitX.calculate(speed.x * allianceSwitch(blue = 1, red = -1))
                vyMetersPerSecond = ratelimitY.calculate(speed.y * allianceSwitch(blue = 1, red = -1))
                omegaRadiansPerSecond = getRotationalSpeed()
                wasJustUsingHeadingController = false
            }
        }, Robot.period)
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
            .onTrue { Swerve.setGyroAngle(allianceSwitch(blue = Rotation2d(), red = Rotation2d(Math.PI))) }

        controller.start
            .onTrue { RobotController.setAction { Actions.outtake() } }
            .onFalse { RobotController.setAction { Actions.idle() } }

        controller.povRight.onTrue { RobotController.setAction { Actions.feedNote() } }
    }


    private fun getRotationalSpeed(): Double {
        return getTriggerRotationSpeed() //+ getJoystickRotationSpeed()
    }

    private fun getJoystickRotationSpeed(): Double {
        return -controller.rightX * Math.PI * 3
    }

    private fun getTriggerRotationSpeed(): Double {
        val rightAxis = controller.rightTriggerAxis
        val leftAxis = controller.leftTriggerAxis
        return ((rightAxis.pow(1.0).withSign(rightAxis) * -1) + leftAxis.pow(1.0).withSign(leftAxis)) * Math.toRadians(270.0)
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
        linearMagnitude = linearMagnitude.pow(2).withSign(linearMagnitude)

        // Calcaulate new linear velocity
        val linearVelocity =
            Pose2d(Translation2d(), linearDirection)
                .transformBy(Transform2d(linearMagnitude, 0.0, Rotation2d()))
                .translation

        return linearVelocity.times(4.0)
    }
}