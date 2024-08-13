package org.team9432.auto

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.team9432.lib.util.ChoreoUtil.getAutoFlippedInitialPose
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds

object Auto {
    suspend fun scoreNote(note: ChoreoTrajectory, spindown: Boolean, simPickupDelay: Duration = 0.75.seconds) = coroutineScope {
        // The driving and shooting are run in separate coroutines so they can happen at the same time

        // This one just handles the driving
        launch { Swerve.followChoreo(note) }

        // While the robot is driving, this one collects and shoots the note (likely only after the robot has stopped)
        launch {
            // Intake and then pull the note in
            AutoActions.intake(simPickupDelay = simPickupDelay)
            AutoActions.shoot(spindown)
        }
    }

    private suspend fun scorePreload() {
        AutoActions.shoot(spindown = false)
    }

    suspend fun runFourNote() {
        val trajectories = Choreo.getTrajectoryGroup("4AndNothing")

        val (ampNote, speakerNote, stageNote) = trajectories

        Swerve.seedFieldRelative(trajectories.first().getAutoFlippedInitialPose())

        scorePreload()

        scoreNote(ampNote, spindown = false)
        scoreNote(speakerNote, spindown = false)
        scoreNote(stageNote, spindown = true)
    }

    suspend fun runFourNoteCenter() {
        val trajectories = Choreo.getTrajectoryGroup("4AndCenter")

        val (ampNote, speakerNote, stageNote, center) = trajectories

        Swerve.seedFieldRelative(trajectories.first().getAutoFlippedInitialPose())

        scorePreload()

        scoreNote(ampNote, spindown = false)
        scoreNote(speakerNote, spindown = false)
        scoreNote(stageNote, spindown = true)

        Swerve.followChoreo(center)
    }
}