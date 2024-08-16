package org.team9432.auto

import com.choreo.lib.ChoreoTrajectory
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.Actions
import org.team9432.Beambreaks
import org.team9432.NoteVisualizer
import org.team9432.lib.coroutines.await
import org.team9432.lib.coroutines.parallel
import org.team9432.lib.util.ChoreoUtil.getAutoFlippedInitialPose
import org.team9432.lib.util.simDelay
import org.team9432.resources.Loader
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration.Companion.seconds

object FourNote {
    suspend fun runFourNote() {
        val trajectories = ChoreoTrajectories.fourAndNothing
        val (firstPath, ampNote, speakerNote, stageNote) = trajectories

        Swerve.seedFieldRelative(trajectories.first().getAutoFlippedInitialPose())

        Shooter.setState(Shooter.State.VISION_SHOOT)
        parallel(
            { Swerve.followChoreo(firstPath) },
            { simDelay(1.seconds) }
        )
        shootNote()

        scoreNote(ampNote)
        scoreNote(speakerNote)
        scoreNote(stageNote)

        Shooter.setState(Shooter.State.IDLE)
    }

    suspend fun runFourNoteCenter() {
        val trajectories = ChoreoTrajectories.fourAndCenter
        val (firstPath, ampNote, speakerNote, stageNote, center) = trajectories

        Swerve.seedFieldRelative(trajectories.first().getAutoFlippedInitialPose())

        Shooter.setState(Shooter.State.VISION_SHOOT)
        parallel(
            { Swerve.followChoreo(firstPath) },
            { simDelay(1.seconds) }
        )
        shootNote()

        scoreNote(ampNote)
        scoreNote(speakerNote)
        scoreNote(stageNote)

        Shooter.setState(Shooter.State.IDLE)
        Swerve.followChoreo(center)
    }

    private suspend fun scoreNote(ampNote: ChoreoTrajectory) = coroutineScope {
        val intakingJob = launch { Actions.intake() }
        Swerve.followChoreo(ampNote)

        if (!Beambreaks.hasNote) {
            intakingJob.cancelAndJoin()
        } else if (Beambreaks.hasNote) {
            shootNote()
        }
    }

    private suspend fun shootNote() {
        await { Shooter.isReadyToShootSpeaker() }
        Loader.setState(Loader.State.LOAD)
        NoteVisualizer.animateShoot()
        delay(0.5.seconds)
        Beambreaks.upper.awaitClear(simDelay = 0.5.seconds)
        Beambreaks.lower.setSimClear()
        Loader.setState(Loader.State.IDLE)
    }
}