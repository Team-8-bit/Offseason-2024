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
    private suspend fun shoot() {
        Actions.preshootPullNote()
        Shooter.set(Shooter.State.SHOOT)
        delay(3.seconds)
        Actions.shootAndSpinDown()
    }

    suspend fun runFourNote() {
        use(Swerve, Intake, Loader, Shooter, cancelConflicts = true) {
            val trajectories = Choreo.getTrajectoryGroup(ChoreoPaths.PATH_4_AND_NOTHING)

            val (ampNote, speakerNote, stageNote) = trajectories

            Swerve.swerve.seedFieldRelative(trajectories.first().initialPose)

            // Shoot preload
            shoot()

            // Collect and shoot the amp note
            Actions.startIntaking()
            Swerve.followChoreo(ampNote)
            Actions.stopIntaking()
            shoot()

            // Repeat for the last two
            Actions.startIntaking()
            Swerve.followChoreo(speakerNote)
            Actions.stopIntaking()
            shoot()

            Actions.startIntaking()
            Swerve.followChoreo(stageNote)
            Actions.stopIntaking()
            shoot()
        }
    }
}