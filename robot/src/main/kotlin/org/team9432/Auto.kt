package org.team9432

import com.choreo.lib.Choreo
import kotlinx.coroutines.delay
import org.team9432.lib.resource.use
import org.team9432.resources.Intake
import org.team9432.resources.Loader
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration.Companion.seconds

object Auto {
    fun startIntaking() {
        Intake.setState(Intake.State.INTAKE)
        Loader.setState(Loader.State.LOAD)
    }

    private fun stopIntaking() {
        Intake.setState(Intake.State.IDLE)
        Loader.setState(Loader.State.IDLE)
    }

    private suspend fun shoot() {
        Loader.setState(Loader.State.REVERSE)
        delay(0.15.seconds)
        Loader.setState(Loader.State.IDLE)
        Shooter.setState(Shooter.State.SHOOT)
        delay(3.seconds)
        Loader.setState(Loader.State.LOAD)
        delay(1.seconds)
        Shooter.setState(Shooter.State.IDLE)
        Loader.setState(Loader.State.IDLE)
    }

    suspend fun runFourNote() {
        use(Swerve, Intake, Loader, Shooter, cancelConflicts = true) {
            val trajectories = Choreo.getTrajectoryGroup("4AndNothing")

            val (ampNote, speakerNote, stageNote) = trajectories

            Swerve.swerve.seedFieldRelative(trajectories.first().initialPose)

            // Shoot preload
            shoot()

            // Collect and shoot the amp note
            startIntaking()
            Swerve.followChoreo(ampNote)
            stopIntaking()
            shoot()

            // Repeat for the last two
            startIntaking()
            Swerve.followChoreo(speakerNote)
            stopIntaking()
            shoot()

            startIntaking()
            Swerve.followChoreo(stageNote)
            stopIntaking()
            shoot()
        }
    }
}