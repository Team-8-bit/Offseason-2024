package org.team9432

import edu.wpi.first.wpilibj.GenericHID
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.lib.coroutines.RobotScope
import org.team9432.lib.resource.use
import org.team9432.oi.Controls
import org.team9432.resources.Intake
import org.team9432.resources.Loader
import org.team9432.resources.Shooter
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

object Actions {
    suspend fun preshootPullNote() {
        use(Loader) {
            Loader.setState(Loader.State.REVERSE)
            delay(0.15.seconds)
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

    suspend fun shoot() {
        use(Loader) {
            Loader.setState(Loader.State.LOAD)
            delay(0.5.seconds)
            Loader.setState(Loader.State.IDLE)
        }
    }

    suspend fun pullNoteAndSpinUpTo(state: Shooter.State) {
        use(Shooter, Loader) {
            preshootPullNote()
            Shooter.setState(state)
        }
    }

    suspend fun runIntake(simDelay: Duration = Duration.INFINITE) {
        use(Intake, Loader) {
            Intake.setState(Intake.State.INTAKE)
            Loader.setState(Loader.State.INTAKE)
        }

        while (!Loader.lowerBeambreak.isTripped()) {
            delay(20.milliseconds)

            if (!Controls.intakeButton.invoke()) {
                Intake.setState(Intake.State.IDLE)
                Loader.setState(Loader.State.IDLE)
                return
            }
        }

        use(Intake, Loader) {
            RobotScope.launch {
                Controls.controller.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
                delay(2.seconds)
                Controls.controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
            }

            Intake.setState(Intake.State.LOAD)
            Loader.setState(Loader.State.LOAD)
            Loader.upperBeambreak.awaitTripped()
            Intake.setState(Intake.State.IDLE)
        }

        use(Loader) {
            Loader.setState(Loader.State.REVERSE)
            delay(0.15.seconds)
            Loader.setState(Loader.State.IDLE)

            Loader.setState(Loader.State.LOAD)
            Loader.upperBeambreak.awaitTripped()
            Loader.setState(Loader.State.IDLE)
        }
    }

//    suspend fun stopIntaking() {
//        use(Intake, Loader) {
//            Intake.setState(Intake.State.IDLE)
//            Loader.setState(Loader.State.IDLE)
//        }
//    }
}