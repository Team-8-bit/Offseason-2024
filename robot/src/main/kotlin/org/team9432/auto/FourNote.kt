package org.team9432.auto

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.Actions
import org.team9432.Beambreaks
import org.team9432.auto.paths.FourNotePaths
import org.team9432.auto.types.AutoType
import org.team9432.lib.coroutines.parallel
import org.team9432.lib.util.ChoreoUtil.getAutoFlippedInitialPose
import org.team9432.lib.util.simDelay
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration.Companion.seconds

object FourNote {
    suspend fun run(auto: AutoType.FourNote) {
        val trajectories = FourNotePaths.generate(auto).map { it.name }.map { Choreo.getTrajectory(it) }

        if (auto.centerNote == null) {
            val (firstPath, firstNote, secondNote, thirdNote) = trajectories

            runGenericFourNote(firstPath, firstNote, secondNote, thirdNote)
        } else {
            val (firstPath, firstNote, secondNote, thirdNote, centerNote) = trajectories

            runGenericFourAndCenter(firstPath, firstNote, secondNote, thirdNote, centerNote)
        }

        Shooter.setState(Shooter.State.IDLE)
    }

//    suspend fun runFourNoteCenter() {
//        val trajectories = ChoreoTrajectories.fourAndCenter
//        val (firstPath, ampNote, speakerNote, stageNote, center) = trajectories
//
//        runGenericFourNote(firstPath, ampNote, speakerNote, stageNote)
//
//        Shooter.setState(Shooter.State.IDLE)
//        Swerve.followChoreo(center)
//    }
//
//    suspend fun runCenter54() {
//        val trajectories = ChoreoTrajectories.center54
//        val (note5, note4) = trajectories
//
//        runGenericDoubleCenter(note5, note4)
//
//        Shooter.setState(Shooter.State.IDLE)
//    }
//
//    suspend fun runCenter12() {
//        val trajectories = ChoreoTrajectories.center12
//        val (note1, note2) = trajectories
//
//        runGenericDoubleCenter(note1, note2)
//
//        Shooter.setState(Shooter.State.IDLE)
//    }
//
//    suspend fun runFourAndCenter1() {
//        val trajectories = ChoreoTrajectories.fourAndCenter1
//        val (firstPath, ampNote, speakerNote, stageNote, centerNote) = trajectories
//
//        runGenericFourAndCenter(firstPath, ampNote, speakerNote, stageNote, centerNote)
//    }
//
//    suspend fun runFourAndCenter2() {
//        val trajectories = ChoreoTrajectories.fourAndCenter2
//        val (firstPath, ampNote, speakerNote, stageNote, centerNote) = trajectories
//
//        runGenericFourAndCenter(firstPath, ampNote, speakerNote, stageNote, centerNote)
//    }
//
//    suspend fun runAmpAndCenter12() {
//        val trajectories = ChoreoTrajectories.ampAndCenter12
//        val (ampNote, note1, note2) = trajectories
//
//        Swerve.seedFieldRelative(ampNote.getAutoFlippedInitialPose())
//
//        Shooter.setState(Shooter.State.VISION_SHOOT)
//
//        simDelay(1.seconds) // Fake flywheel spinup
//        Actions.visionShoot(spindown = false)
//
//        scoreNotes(ampNote, note1, note2)
//    }

    private suspend fun runGenericFourAndCenter(
        firstPath: ChoreoTrajectory,
        firstNote: ChoreoTrajectory,
        secondNote: ChoreoTrajectory,
        thirdNote: ChoreoTrajectory,
        centerNote: ChoreoTrajectory,
    ) {
        runGenericFourNote(firstPath, firstNote, secondNote, thirdNote)
        scoreNote(centerNote)

        Shooter.setState(Shooter.State.IDLE)
    }

    private suspend fun runGenericFourNote(
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

    private suspend fun runGenericDoubleCenter(
        firstNote: ChoreoTrajectory,
        secondNote: ChoreoTrajectory,
    ) {
        Swerve.seedFieldRelative(firstNote.getAutoFlippedInitialPose())

        Shooter.setState(Shooter.State.VISION_SHOOT)

        simDelay(1.seconds) // Fake flywheel spinup
        Actions.visionShoot(spindown = false)

        scoreNotes(firstNote, secondNote)
    }

    private suspend fun scoreNotes(vararg notes: ChoreoTrajectory) = notes.forEach { scoreNote(it) }

    private suspend fun scoreNote(ampNote: ChoreoTrajectory) = coroutineScope {
        val intakingJob = launch { Actions.intake() }
        Swerve.followChoreo(ampNote)

        if (!Beambreaks.hasNote) {
            intakingJob.cancelAndJoin()
        } else if (Beambreaks.hasNote) {
            delay(0.25.seconds)
            Actions.visionShoot(spindown = false)
        }
    }
}