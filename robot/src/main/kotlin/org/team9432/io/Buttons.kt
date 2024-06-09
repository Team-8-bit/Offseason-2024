package org.team9432.io

import org.team9432.lib.input.XboxController
import org.team9432.lib.resource.use
import org.team9432.lib.resource.useUntilCancelled
import org.team9432.resources.Indexer
import org.team9432.resources.Intake
import org.team9432.resources.Shooter

object Buttons {
    private val controller = XboxController(0)

    fun bind() {
        controller.a
            .onTrue { Intake.state = Intake.State.INTAKE }
            .onFalse { Intake.state = Intake.State.IDLE }

        controller.b
            .whileTrue {
                useUntilCancelled(Intake) {
                    Intake.state = Intake.State.INTAKE
                }
            }

        controller.rightTrigger.onTrue {
            use(Shooter, Indexer) {
                Shooter.spinUpToState(Shooter.State.SHOOT)
                Indexer.state = Indexer.State.LOAD
                Indexer.sensor.awaitUnbroken()
            }
        }

        controller.leftTrigger.whileTrue {
            use(Intake, Indexer) {
                Intake.state = Intake.State.INTAKE
                Indexer.state = Indexer.State.LOAD
                Indexer.sensor.awaitTripped()
            }
        }
    }
}