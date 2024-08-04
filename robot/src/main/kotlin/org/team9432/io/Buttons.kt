package org.team9432.io

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import kotlinx.coroutines.delay
import org.team9432.lib.input.XboxController
import org.team9432.lib.resource.use
import org.team9432.resources.Intake
import org.team9432.resources.Loader
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.math.pow
import kotlin.math.withSign
import kotlin.time.Duration.Companion.seconds

object Buttons {
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
        controller.y.onTrue {
            use(Intake, Shooter, Loader, Swerve, cancelConflicts = true) {
                Intake.setState(Intake.State.IDLE)
                Shooter.setState(Shooter.State.IDLE)
                Loader.setState(Loader.State.IDLE)
            }
        }

        controller.leftBumper
            .onTrue {
                use(Intake, Loader, cancelConflicts = true) {
                    Intake.setState(Intake.State.INTAKE)
                    Loader.setState(Loader.State.LOAD)
                }
            }
            .onFalse {
                use(Intake, Loader, cancelConflicts = true) {
                    Intake.setState(Intake.State.IDLE)
                    Loader.setState(Loader.State.IDLE)
                }
            }

        controller.b
            .whileTrue {
                use(Shooter, Loader, cancelConflicts = true) {
                    Loader.setState(Loader.State.REVERSE)
                    delay(0.15.seconds)
                    Loader.setState(Loader.State.IDLE)
                    Shooter.setState(Shooter.State.VISION_SHOOT)
                }
            }
            .onFalse {
                use(Shooter, Loader, cancelConflicts = true) {
                    Loader.setState(Loader.State.LOAD)
                    delay(1.seconds)
                    Shooter.setState(Shooter.State.IDLE)
                    Loader.setState(Loader.State.IDLE)
                }
            }

        controller.x
            .whileTrue {
                use(Shooter, Loader, cancelConflicts = true) {
                    Loader.setState(Loader.State.REVERSE)
                    delay(0.15.seconds)
                    Loader.setState(Loader.State.IDLE)
                    Shooter.setState(Shooter.State.SUBWOOFER)
                }
            }
            .onFalse {
                use(Shooter, Loader, cancelConflicts = true) {
                    Loader.setState(Loader.State.LOAD)
                    delay(1.seconds)
                    Shooter.setState(Shooter.State.IDLE)
                    Loader.setState(Loader.State.IDLE)
                }
            }

        controller.a
            .whileTrue {
                use(Shooter, Loader, cancelConflicts = true) {
                    Loader.setState(Loader.State.REVERSE)
                    delay(0.15.seconds)
                    Loader.setState(Loader.State.IDLE)
                    Shooter.setState(Shooter.State.AMP)
                }
            }
            .onFalse {
                use(Shooter, Loader, cancelConflicts = true) {
                    Loader.setState(Loader.State.LOAD)
                    delay(1.seconds)
                    Shooter.setState(Shooter.State.IDLE)
                    Loader.setState(Loader.State.IDLE)
                }
            }

//        controller.x.onTrue { Orchestra.loadAndPlay("mario.chrp") }
//        controller.y.onTrue { Orchestra.loadAndPlay("megalovania.chrp") }

        //controller.a.onTrue { Swerve.followChoreo("NewPath") }
        controller.back.onTrue { Swerve.swerve.seedFieldRelative() }

//        controller.x
//            .whileTrue {
//                use(Shooter) {
//                    Shooter.setState(Shooter.State.DASHBOARD_SPEEDS)
//                }
//            }
//            .onFalse {
//                use(Shooter, cancelConflicts = true) {
//                    Shooter.setState(Shooter.State.IDLE)
//                }
//            }

//        val tests = Shooter.getSysId()
//        val (quasistaticForward, quasistaticReverse, dynamicForward, dynamicReverse) = tests

//        controller.a.onTrue(quasistaticForward)
//        controller.b.onTrue(quasistaticReverse)
//        controller.x.onTrue(dynamicForward)
//        controller.y.onTrue(dynamicReverse)
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