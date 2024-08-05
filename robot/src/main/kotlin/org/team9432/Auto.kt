package org.team9432

import com.choreo.lib.Choreo
import kotlinx.coroutines.delay
import org.team9432.generated.ChoreoPaths
import org.team9432.lib.resource.use
import org.team9432.resources.Intake
import org.team9432.resources.Loader
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration.Companion.seconds

object Auto {
    fun startIntaking() {
        Intake.set(Intake.State.INTAKE)
        Loader.set(Loader.State.LOAD)
    }

    private fun stopIntaking() {
        Intake.set(Intake.State.IDLE)
        Loader.set(Loader.State.IDLE)
    }

    private suspend fun shoot() {
        Loader.set(Loader.State.REVERSE)
        delay(0.15.seconds)
        Loader.set(Loader.State.IDLE)
        Shooter.setState(Shooter.State.SHOOT)
        delay(3.seconds)
        Loader.set(Loader.State.LOAD)
        delay(1.seconds)
        Shooter.setState(Shooter.State.IDLE)
        Loader.set(Loader.State.IDLE)
    }

    suspend fun runFourNote() {
        use(Swerve, Intake, Loader, Shooter, cancelConflicts = true) {
            val trajectories = Choreo.getTrajectoryGroup(ChoreoPaths.PATH_4_AND_NOTHING)

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