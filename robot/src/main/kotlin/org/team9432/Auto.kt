package org.team9432

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.team9432.generated.ChoreoPaths
import org.team9432.lib.resource.use
import org.team9432.lib.util.getAutoFlippedInitialPose
import org.team9432.resources.Intake
import org.team9432.resources.Loader
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration.Companion.seconds

object Auto {
    suspend fun scoreNote(note: ChoreoTrajectory) = coroutineScope {
        // The driving and shooting are run in separate coroutines so they can happen at the same time

        // This one just handles the driving
        launch { Swerve.followChoreo(note) }

        // While the robot is driving, this one collects and shoots the note (likely only after the robot has stopped)
        launch {
            // Intake and then pull the note in
            Actions.runIntakeUntilCollect(simDelay = 0.75.seconds)
            Actions.preshootPullNote()
            Shooter.setState(Shooter.State.VISION_SHOOT)
            Shooter.awaitReady()
            Actions.shootAndSpinDown()
        }
    }

    private suspend fun scorePreload() {
        // Spin up shooter
        Shooter.setState(Shooter.State.VISION_SHOOT)
        // Pull the preload in
        Actions.preshootPullNote()
        // Wait for the shooter to spin up
        Shooter.awaitReady()
        // Shoot preload
        Actions.shootAndSpinDown()
    }

    suspend fun runFourNote() {
        use(Swerve, Intake, Loader, Shooter) {
            val trajectories = Choreo.getTrajectoryGroup(ChoreoPaths.PATH_4_AND_NOTHING)

            val (ampNote, speakerNote, stageNote) = trajectories

            Swerve.seedFieldRelative(trajectories.first().getAutoFlippedInitialPose())

            scorePreload()

            scoreNote(ampNote)
            scoreNote(speakerNote)
            scoreNote(stageNote)
        }
    }

    suspend fun runFourNoteCenter() {
        use(Swerve, Intake, Loader, Shooter) {
            val trajectories = Choreo.getTrajectoryGroup(ChoreoPaths.PATH_4_AND_CENTER)

            val (ampNote, speakerNote, stageNote, center) = trajectories

            Swerve.seedFieldRelative(trajectories.first().getAutoFlippedInitialPose())

            scorePreload()

            scoreNote(ampNote)
            scoreNote(speakerNote)
            scoreNote(stageNote)

            Swerve.followChoreo(center)
        }
    }
}