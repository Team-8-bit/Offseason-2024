package org.team9432.io

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest
import kotlinx.coroutines.delay
import org.team9432.lib.input.XboxController
import org.team9432.lib.resource.use
import org.team9432.resources.Indexer
import org.team9432.resources.Intake
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
            .withRotationalRate(calculateRotationalSpeed())


    init {
        controller.leftBumper
            .onTrue {
                Intake.set(Intake.State.INTAKE)
                Indexer.set(Indexer.State.LOAD)
            }
            .onFalse {
                Intake.set(Intake.State.IDLE)
                Indexer.set(Indexer.State.IDLE)
            }

        controller.b
            .whileTrue {
                use(Shooter, Indexer) {
                    Indexer.set(Indexer.State.REVERSE)
                    delay(0.15.seconds)
                    Indexer.set(Indexer.State.IDLE)
                    Shooter.setState(Shooter.State.SHOOT)
                }
            }
            .onFalse {
                use(Shooter, Indexer, cancelConflicts = true) {
                    Indexer.set(Indexer.State.LOAD)
                    delay(1.seconds)
                    Shooter.setState(Shooter.State.IDLE)
                    Indexer.set(Indexer.State.IDLE)
                }
            }

        controller.a
            .whileTrue {
                use(Shooter, Indexer) {
                    Indexer.set(Indexer.State.REVERSE)
                    delay(0.15.seconds)
                    Indexer.set(Indexer.State.IDLE)
                    Shooter.setState(Shooter.State.AMP)
                }
            }
            .onFalse {
                use(Shooter, Indexer, cancelConflicts = true) {
                    Indexer.set(Indexer.State.LOAD)
                    delay(1.seconds)
                    Shooter.setState(Shooter.State.IDLE)
                    Indexer.set(Indexer.State.IDLE)
                }
            }

//        controller.x.onTrue { Orchestra.loadAndPlay("mario.chrp") }
//        controller.y.onTrue { Orchestra.loadAndPlay("megalovania.chrp") }

        controller.a.onTrue { Swerve.followChoreo("NewPath") }
        controller.back.onTrue { Swerve.swerve.seedFieldRelative() }
    }

    private fun calculateRotationalSpeed(): Double {
        val rightAxis = controller.rightTriggerAxis
        val leftAxis = controller.leftTriggerAxis
        return ((rightAxis.pow(2) * -1) + leftAxis.pow(2)) * Math.toRadians(360.0)
    }

    private fun getTranslationalSpeed(rawJoystick: Double): Double {
        val speedSquared = (rawJoystick * rawJoystick).withSign(rawJoystick)
        return speedSquared * if (controller.rightBumper.invoke()) 3.0 else 5.0
    }
}