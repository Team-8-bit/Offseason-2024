package org.team9432.io

import kotlinx.coroutines.delay
import org.team9432.Orchestra
import org.team9432.lib.input.XboxController
import org.team9432.lib.resource.use
import org.team9432.resources.Indexer
import org.team9432.resources.Intake
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration.Companion.seconds

object Buttons {
    val controller = XboxController(0)

    fun bind() {
        controller.leftBumper
            .onTrue {
                Intake.set(Intake.State.INTAKE)
                Indexer.set(Indexer.State.LOAD)
            }
            .onFalse {
                Intake.set(Intake.State.IDLE)
                Indexer.set(Indexer.State.IDLE)
            }

        controller.rightTrigger
            .onTrue {
//            use(Shooter, Indexer) {
                Shooter.spinUpToState(Shooter.State.SHOOT)
                delay(1.seconds)
                Indexer.set(Indexer.State.LOAD)
//            }
            }
            .onFalse {
                Shooter.spinUpToState(Shooter.State.IDLE)
                Indexer.set(Indexer.State.IDLE)
            }

        controller.leftTrigger.whileTrue {
            use(Intake, Indexer, name = "Driver Intake") {
                Intake.set(Intake.State.INTAKE)
                Indexer.set(Indexer.State.LOAD)
                Indexer.sensor.awaitTripped()
            }
        }

        controller.x.onTrue { Orchestra.loadAndPlay("mario.chrp") }
        controller.y.onTrue { Orchestra.loadAndPlay("megalovania.chrp") }

        controller.a.onTrue { Swerve.followChoreo("Path One") }
    }
}