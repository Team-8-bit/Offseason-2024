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
            Loader.setState(Loader.State.REVERSE)
            delay(0.15.seconds)
            Loader.setState(Loader.State.IDLE)
        }
    }

    suspend fun startIntaking() {
        use(Intake, Loader) {
            Intake.setState(Intake.State.INTAKE)
            Loader.setState(Loader.State.LOAD)
        }
    }

    suspend fun stopIntaking() {
        use(Intake, Loader) {
            Intake.setState(Intake.State.IDLE)
            Loader.setState(Loader.State.IDLE)
        }
    }

    suspend fun shootAndSpinDown() {
        use(Shooter, Loader) {
            Loader.setState(Loader.State.LOAD)
            delay(1.seconds)
            Shooter.setState(Shooter.State.IDLE)
            Loader.setState(Loader.State.IDLE)
        }
    }

    suspend fun pullNoteAndSpinUpTo(state: Shooter.State) {
        use(Shooter, Loader) {
            preshootPullNote()
            Shooter.setState(state)
        }
    }
}