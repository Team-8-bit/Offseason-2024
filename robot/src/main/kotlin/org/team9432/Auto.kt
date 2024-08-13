package org.team9432

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.team9432.generated.ChoreoPaths
import org.team9432.lib.coroutines.await
import org.team9432.lib.util.ChoreoUtil.getAutoFlippedInitialPose
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve

object Auto {
    suspend fun scoreNote(note: ChoreoTrajectory) = coroutineScope {
        // The driving and shooting are run in separate coroutines so they can happen at the same time

        // This one just handles the driving
        launch { Swerve.followChoreo(note) }

        // While the robot is driving, this one collects and shoots the note (likely only after the robot has stopped)
        launch {
            // Intake and then pull the note in
            Actions.runIntake()
//            Actions.preshootPullNote()
            Shooter.setState(Shooter.State.VISION_SHOOT)
            await { Shooter.isReadyToShootSpeaker() }
//            Actions.shootAndSpinDown()
        }
    }

    private suspend fun scorePreload() {
        // Spin up shooter
        Shooter.setState(Shooter.State.VISION_SHOOT)
        // Pull the preload in
//        Actions.preshootPullNote()
        // Wait for the shooter to spin up
        await { Shooter.isReadyToShootSpeaker() }
        // Shoot preload
//        Actions.shootAndSpinDown()
    }

    suspend fun runFourNote() {
        val trajectories = Choreo.getTrajectoryGroup(ChoreoPaths.PATH_4_AND_NOTHING)

        val (ampNote, speakerNote, stageNote) = trajectories

        Swerve.seedFieldRelative(trajectories.first().getAutoFlippedInitialPose())

        scorePreload()

        scoreNote(ampNote)
        scoreNote(speakerNote)
        scoreNote(stageNote)
    }

    suspend fun runFourNoteCenter() {
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