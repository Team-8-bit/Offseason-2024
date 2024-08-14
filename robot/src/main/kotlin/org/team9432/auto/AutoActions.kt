package org.team9432.auto

import kotlinx.coroutines.delay
import org.team9432.NoteVisualizer
import org.team9432.Robot
import org.team9432.lib.coroutines.await
import org.team9432.resources.Intake
import org.team9432.resources.Loader
import org.team9432.resources.Shooter
import kotlin.time.Duration.Companion.seconds

object AutoActions {
    suspend fun intake() {
        Intake.setState(Intake.State.INTAKE)
        Loader.setState(Loader.State.INTAKE)

        if (Robot.isSimulated) {
            NoteVisualizer.awaitNotePickup()
            Loader.lowerBeambreak.setSimTripped()
        } else {
            Loader.lowerBeambreak.awaitTripped()
        }

        Intake.setState(Intake.State.LOAD)
        Loader.setState(Loader.State.LOAD)
        Loader.upperBeambreak.awaitTripped(simDelay = 0.3.seconds)
        Intake.setState(Intake.State.IDLE)

        repeat(2) {
            NoteVisualizer.animateAlign()
            Loader.setState(Loader.State.REVERSE)
            delay(0.15.seconds)
            Loader.upperBeambreak.setSimClear()

            Loader.setState(Loader.State.LOAD)
            Loader.upperBeambreak.awaitTripped(simDelay = 0.2.seconds)
            Loader.setState(Loader.State.IDLE)
        }
    }

    suspend fun shoot(spindown: Boolean) {
        Shooter.setState(Shooter.State.VISION_SHOOT)
        await { Shooter.isReadyToShootSpeaker() }
        Loader.setState(Loader.State.LOAD)
        NoteVisualizer.animateShoot()

        Loader.upperBeambreak.awaitClear(simDelay = 0.5.seconds)
        Loader.lowerBeambreak.setSimClear()

        delay(0.25.seconds)

        Loader.setState(Loader.State.IDLE)
        if (spindown) Shooter.setState(Shooter.State.IDLE)
    }
}