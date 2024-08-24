package org.team9432.auto.paths

import org.team9432.auto.paths.AutoFieldConstants.CenterNote
import org.team9432.auto.paths.AutoFieldConstants.CenterNote.*
import org.team9432.auto.paths.AutoFieldConstants.CloseNote.*
import org.team9432.choreogenerator.*
import org.team9432.choreogenerator.ChoreoTrajectory.ChoreoTrajectoryBuilder
import org.team9432.lib.unit.degrees
import org.team9432.lib.unit.meters

object FourNote {
    private val FOUR_NOTE_START = Position(1.34.meters, AutoFieldConstants.speakerYCoordinate, 180.0.degrees)
    private val FOUR_NOTE_SHOT = Position(1.7.meters, AutoFieldConstants.speakerYCoordinate, 180.0.degrees)
    private val FOUR_NOTE_AMP_ALIGN = AutoFieldConstants.ampNote.copy().moveTowards(FOUR_NOTE_SHOT, 0.75.meters).pointAwayFrom(AutoFieldConstants.ampNote)
    private val FOUR_NOTE_AMP_PICKUP = AutoFieldConstants.ampNote.copy().moveTowards(FOUR_NOTE_SHOT, 0.15.meters).pointAwayFrom(AutoFieldConstants.ampNote)
    private val FOUR_NOTE_SPEAKER_PICKUP = Position(2.88.meters, AutoFieldConstants.speakerYCoordinate, 180.0.degrees)
    private val FOUR_NOTE_STAGE_ALIGN = AutoFieldConstants.stageNote.copy().moveX(-0.6.meters).pointAwayFrom(AutoFieldConstants.stageNote)
    private val FOUR_NOTE_STAGE_PICKUP = AutoFieldConstants.stageNote.copy().moveX(-0.2.meters).pointAwayFrom(AutoFieldConstants.stageNote)
    private val FOUR_NOTE_STAGE_EXIT = AutoFieldConstants.stageNote.copy().move(x = -0.5.meters, y = 0.5.meters, heading = 180.degrees)//.pointAwayFrom(AutoFieldConstants.stageNote)

    private val FOUR_NOTE_ABOVE_STAGE_DRIVE = Position(5.5.meters, 6.5.meters, 180.0.degrees)
    private val FOUR_NOTE_THROUGH_STAGE_DRIVE_CLOSE = Position(4.4.meters, 5.meters, 180.0.degrees)
    private val FOUR_NOTE_THROUGH_STAGE_DRIVE_FAR = Position(6.meters, 4.meters, 180.0.degrees)

    fun getAllPossibilities(): List<ChoreoTrajectory> {
        val trajectories = mutableListOf<ChoreoTrajectory>()
        val centerNoteOptions = listOf(null, ONE, TWO, THREE)

        for (centerNote in centerNoteOptions) {
            for (reversed in listOf(true, false)) {
                val centerNoteName = centerNote?.readableName ?: "Nothing"
                val reversedName = if (reversed) "Reversed" else ""
                val name = "4And$centerNoteName$reversedName"

                val traj = generateFourNote(name, reversed, centerNote)
                trajectories.add(traj)
            }
        }

        return trajectories
    }

    fun generateFourNote(name: String, reversed: Boolean, centerNote: CenterNote?) = ChoreoTrajectory.new(name) {
        preload()

        val noteList = if (reversed) listOf(STAGE, SPEAKER, AMP) else listOf(AMP, SPEAKER, STAGE)

        noteList.forEach { closeNote(it) }

        if (centerNote != null) {
            centerNote(centerNote)
        }
    }

    private fun getCenterNoteAlignPose(note: CenterNote): Position {
        val notePose = AutoFieldConstants.getNotePose(note)
        return notePose.copy().moveX(-.6.meters).pointAwayFrom(notePose)
    }

    private fun getCenterNoteIntakePose(note: CenterNote): Position {
        val notePose = AutoFieldConstants.getNotePose(note)
        return notePose.copy().moveX(-.2.meters).pointAwayFrom(notePose)
    }

    private fun ChoreoTrajectoryBuilder.centerNote(note: CenterNote) {
        if (note != ONE && note != TWO && note != THREE) throw UnsupportedOperationException("Note ${note.name} is not supported!")

        val travelPath: ((Boolean) -> Unit)? = when (note) {
            ONE -> null
            TWO -> { _ -> addTranslationWaypoint(FOUR_NOTE_ABOVE_STAGE_DRIVE) }
            THREE -> { returning ->
                if (returning) {
                    addTranslationWaypoint(FOUR_NOTE_THROUGH_STAGE_DRIVE_FAR)
                    addTranslationWaypoint(FOUR_NOTE_THROUGH_STAGE_DRIVE_CLOSE)
                } else {
                    addTranslationWaypoint(FOUR_NOTE_THROUGH_STAGE_DRIVE_CLOSE)
                    addTranslationWaypoint(FOUR_NOTE_THROUGH_STAGE_DRIVE_FAR)
                }
            }

            else -> throw UnsupportedOperationException()
        }

        travelPath?.invoke(/*returning =*/false)

        val alignPose = addPoseWaypoint(getCenterNoteAlignPose(note))
        val intakePose = addPoseWaypoint(getCenterNoteIntakePose(note))
        addConstraint(StraightLine(ConstraintScope.betweenWaypoints(alignPose, intakePose)))

        travelPath?.invoke(/*returning =*/true)

        addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)
    }

    private fun ChoreoTrajectoryBuilder.preload() {
        addPoseWaypoint(FOUR_NOTE_START)
        addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)
    }

    private fun ChoreoTrajectoryBuilder.closeNote(note: AutoFieldConstants.CloseNote) {
        when (note) {
            AMP -> {
                addPoseWaypoint(FOUR_NOTE_AMP_ALIGN)
                addPoseWaypoint(FOUR_NOTE_AMP_PICKUP)
                addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)
            }

            SPEAKER -> {
                addPoseWaypoint(FOUR_NOTE_SPEAKER_PICKUP)
                addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)
            }

            STAGE -> {
                val straightLineStart = addPoseWaypoint(FOUR_NOTE_STAGE_ALIGN)
                val straightLineEnd = addPoseWaypoint(FOUR_NOTE_STAGE_PICKUP)
                addConstraint(StraightLine(ConstraintScope.betweenWaypoints(straightLineStart, straightLineEnd)))
                addTranslationWaypoint(FOUR_NOTE_STAGE_EXIT)
                addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)
            }
        }
    }
}