package org.team9432.io

import org.team9432.Orchestra
import org.team9432.lib.input.XboxController
import org.team9432.lib.resource.use
import org.team9432.lib.resource.useUntilCancelled
import org.team9432.resources.Indexer
import org.team9432.resources.Intake
import org.team9432.resources.Shooter

object Buttons {
    val controller = XboxController(0)

    fun bind() {
        controller.a
            .onTrue { Intake.set(Intake.State.INTAKE) }
            .onFalse { Intake.set(Intake.State.IDLE) }

        controller.b
            .whileTrue {
                useUntilCancelled(Intake) {
                    Intake.set(Intake.State.INTAKE)
                }
            }

        controller.rightTrigger.onTrue {
            use(Shooter, Indexer) {
                Shooter.spinUpToState(Shooter.State.SHOOT)
                Indexer.set(Indexer.State.LOAD)
                Indexer.sensor.awaitClear()
            }
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
    }
}