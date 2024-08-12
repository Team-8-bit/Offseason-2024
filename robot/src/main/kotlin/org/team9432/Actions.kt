package org.team9432

import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.lib.coroutines.RobotScope
import org.team9432.oi.Controls
import org.team9432.resources.Intake
import org.team9432.resources.Loader
import org.team9432.resources.Shooter
import kotlin.time.Duration.Companion.seconds

object Actions {
    fun idle() {
        Intake.setState(Intake.State.IDLE)
        Loader.setState(Loader.State.IDLE)
        Shooter.setState(Shooter.State.IDLE)
    }

    suspend fun preshootPullNote() {
        Loader.setState(Loader.State.REVERSE)
        delay(0.15.seconds)
        Loader.setState(Loader.State.IDLE)
    }

    suspend fun shootAndSpinDown() {
        Loader.setState(Loader.State.LOAD)
        delay(1.seconds)
        Shooter.setState(Shooter.State.IDLE)
        Loader.setState(Loader.State.IDLE)
    }

    suspend fun pullNoteAndSpinUpTo(state: Shooter.State) {
        preshootPullNote()
        Shooter.setState(state)
    }

    suspend fun runIntake() {
        Intake.setState(Intake.State.INTAKE)
        Loader.setState(Loader.State.INTAKE)

        Loader.lowerBeambreak.awaitTripped(simDelay = 1.seconds)

        RobotScope.launch { Controls.controller.rumbleDuration(2.seconds) }

        RobotController.queueRobotRequest { intakeToLoaderTransition() }
    }

    private suspend fun intakeToLoaderTransition() {
        Intake.setState(Intake.State.LOAD)
        Loader.setState(Loader.State.LOAD)
        Loader.upperBeambreak.awaitTripped(simDelay = 0.3.seconds)
        Intake.setState(Intake.State.IDLE)

        Loader.setState(Loader.State.REVERSE)
        delay(0.15.seconds)
        Loader.setState(Loader.State.IDLE)

        Loader.setState(Loader.State.LOAD)
        Loader.upperBeambreak.awaitTripped(simDelay = 0.2.seconds)
        Loader.setState(Loader.State.IDLE)
    }
}