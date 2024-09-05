package org.team9432

import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.lib.coroutines.await
import org.team9432.oi.Controls
import org.team9432.resources.intake.Intake
import org.team9432.resources.Shooter
import org.team9432.resources.loader.Loader
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

object Actions {
    fun idle() {
        Intake.setState(Intake.State.IDLE)
        Loader.setState(Loader.State.IDLE)
        Shooter.setState(Shooter.State.IDLE)
    }

    fun outtake() {
        Intake.setState(Intake.State.OUTTAKE)
        Loader.setState(Loader.State.REVERSE)
    }

    suspend fun visionShoot(spindown: Boolean = true) {
        Shooter.setState(Shooter.State.VISION_SHOOT)
        await { Shooter.isReadyToShootSpeaker() }
        Loader.setState(Loader.State.LOAD)
        NoteVisualizer.animateShoot()

        Beambreaks.upper.awaitTripped(simDelay = 0.1.seconds) // Make sure the note is actually in the beambreak so the next step doesn't immediately return true
        Beambreaks.upper.awaitClear(simDelay = 0.4.seconds)
        Beambreaks.lower.setSimClear()

        delay(0.25.seconds)

        Loader.setState(Loader.State.IDLE)
        if (spindown) Shooter.setState(Shooter.State.IDLE)
    }

    suspend fun feedNote() {
        Shooter.setState(Shooter.State.FEED_SPEED)
        await { Shooter.flywheelsAtSpeed() }
        Loader.setState(Loader.State.LOAD)
        NoteVisualizer.animateShoot()

        Beambreaks.upper.awaitTripped(simDelay = 0.1.seconds) // Make sure the note is actually in the beambreak so the next step doesn't immediately return true
        Beambreaks.upper.awaitClear(simDelay = 0.4.seconds)
        Beambreaks.lower.setSimClear()

        Loader.setState(Loader.State.IDLE)
        Shooter.setState(Shooter.State.IDLE)
    }

    suspend fun dashboardShoot() {
        Shooter.setState(Shooter.State.DASHBOARD_SPEEDS)
        await { Shooter.flywheelsAtSpeed() }
        delay(0.5.seconds)
        Loader.setState(Loader.State.LOAD)
        NoteVisualizer.animateShoot()

        Beambreaks.upper.awaitTripped(simDelay = 0.1.seconds) // Make sure the note is actually in the beambreak so the next step doesn't immediately return true
        Beambreaks.upper.awaitClear(simDelay = 0.4.seconds)
        Beambreaks.lower.setSimClear()

        Loader.setState(Loader.State.IDLE)
        Shooter.setState(Shooter.State.IDLE)
    }

    suspend fun amp() {
        Shooter.setState(Shooter.State.AMP)
        await { Shooter.flywheelsAtSpeed(rpmTolerance = 100) || Robot.isSimulated }
        delay(0.2.seconds)
        await { !Controls.controller.b.invoke() }
        Loader.setState(Loader.State.LOAD)

        Beambreaks.upper.awaitTripped(simDelay = 0.1.seconds) // Make sure the note is actually in the beambreak so the next step doesn't immediately return true
        Beambreaks.upper.awaitClear(simDelay = 0.4.seconds)
        Beambreaks.lower.setSimClear()
        delay(0.25.seconds)

        NoteVisualizer.clearNote()

        Shooter.setState(Shooter.State.IDLE)
        Loader.setState(Loader.State.IDLE)
    }

    suspend fun intake() {
        Intake.setState(Intake.State.INTAKE)
        Loader.setState(Loader.State.INTAKE)

        if (Robot.isSimulated) {
            NoteVisualizer.awaitNotePickup()
            Beambreaks.lower.setSimTripped()
        } else {
            Beambreaks.lower.awaitTripped()
        }

        if (Robot.mode.isTeleop) {
            Robot.coroutineScope.launch { Controls.controller.rumbleDuration(2.seconds) }
        }

        Intake.setState(Intake.State.LOAD)
        Loader.setState(Loader.State.LOAD)
        Beambreaks.upper.awaitTripped(simDelay = 0.3.seconds, period = 3.milliseconds)
        Intake.setState(Intake.State.IDLE)

        // Align Note
        repeat(1) {
            NoteVisualizer.animateAlign()
            Loader.setState(Loader.State.REVERSE)
            Beambreaks.upper.awaitClear(simDelay = 0.2.seconds, period = 3.milliseconds)

            Loader.setState(Loader.State.LOAD)
            Beambreaks.upper.awaitTripped(simDelay = 0.2.seconds, period = 3.milliseconds)
            Loader.setState(Loader.State.IDLE)
        }
    }
}