package org.team9432

import kotlinx.coroutines.delay
import org.team9432.lib.resource.use
import org.team9432.resources.Intake
import org.team9432.resources.Loader
import org.team9432.resources.Shooter
import kotlin.time.Duration.Companion.seconds

object Actions {
    suspend fun preshootPullNote() {
        use(Loader) {
            Loader.set(Loader.State.REVERSE)
            delay(0.15.seconds)
            Loader.set(Loader.State.IDLE)
        }
    }

    suspend fun startIntaking() {
        use(Intake, Loader) {
            Intake.set(Intake.State.INTAKE)
            Loader.set(Loader.State.LOAD)
        }
    }

    suspend fun stopIntaking() {
        use(Intake, Loader) {
            Intake.set(Intake.State.IDLE)
            Loader.set(Loader.State.IDLE)
        }
    }

    suspend fun shootAndSpinDown() {
        use(Shooter, Loader) {
            Loader.set(Loader.State.LOAD)
            delay(1.seconds)
            Shooter.set(Shooter.State.IDLE)
            Loader.set(Loader.State.IDLE)
        }
    }

    suspend fun pullNoteAndSpinUpTo(state: Shooter.State) {
        use(Shooter, Loader) {
            preshootPullNote()
            Shooter.set(state)
        }
    }
}