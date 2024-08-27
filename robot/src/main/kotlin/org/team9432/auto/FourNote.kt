package org.team9432.auto

import com.choreo.lib.ChoreoTrajectory
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.Actions
import org.team9432.Beambreaks
import org.team9432.auto.paths.FourNotePaths
import org.team9432.auto.types.AutoType
import org.team9432.auto.types.AutoType.FourNote.EndAction.*
import org.team9432.lib.coroutines.parallel
import org.team9432.lib.util.ChoreoUtil
import org.team9432.lib.util.ChoreoUtil.getAutoFlippedInitialPose
import org.team9432.lib.util.simDelay
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration.Companion.seconds

object FourNote {
    suspend fun run(auto: AutoType.FourNote) {
        val trajectories = FourNotePaths.generate(auto).map { it.name }.map { ChoreoUtil.getTrajectoryWithCache(it) }

        when (auto.endAction) {
            DO_NOTHING -> {
                val (firstPath, firstNote, secondNote, thirdNote) = trajectories

                runFourNote(firstPath, firstNote, secondNote, thirdNote)
            }
            SCORE_CENTERLINE -> {
                val (firstPath, firstNote, secondNote, thirdNote, centerNote) = trajectories

                runFourNote(firstPath, firstNote, secondNote, thirdNote)
                scoreNote(centerNote)
            }
            DRIVE_TO_CENTER -> {
                val (firstPath, firstNote, secondNote, thirdNote, driveToCenter) = trajectories

                runFourNote(firstPath, firstNote, secondNote, thirdNote)
                Shooter.setState(Shooter.State.IDLE)
                Swerve.followChoreo(driveToCenter)
            }
        }

        Shooter.setState(Shooter.State.IDLE)
    }

    private suspend fun runFourNote(
        firstPath: ChoreoTrajectory,
        firstNote: ChoreoTrajectory,
        secondNote: ChoreoTrajectory,
        thirdNote: ChoreoTrajectory,
    ) {
        Swerve.seedFieldRelative(firstPath.getAutoFlippedInitialPose())

        Shooter.setState(Shooter.State.VISION_SHOOT)
        parallel(
            { Swerve.followChoreo(firstPath) },
            { simDelay(1.seconds) }
        )
        Actions.visionShoot(spindown = false)

        scoreNotes(firstNote, secondNote, thirdNote)
    }

    private suspend fun scoreNotes(vararg notes: ChoreoTrajectory) = notes.forEach { scoreNote(it) }

    private suspend fun scoreNote(ampNote: ChoreoTrajectory) = coroutineScope {
        val intakingJob = launch { Actions.intake() }
        Swerve.followChoreo(ampNote)

        if (!Beambreaks.hasNote) {
            intakingJob.cancelAndJoin()
        } else if (Beambreaks.hasNote) {
            delay(0.1.seconds)
            Actions.visionShoot(spindown = false)
        }
    }
}