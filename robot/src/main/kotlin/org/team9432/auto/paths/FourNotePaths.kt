package org.team9432.auto.paths

import org.team9432.auto.paths.AutoFieldConstants.CenterNote
import org.team9432.auto.paths.AutoFieldConstants.CenterNote.*
import org.team9432.auto.paths.AutoFieldConstants.CloseNote.*
import org.team9432.auto.types.AutoSegment
import org.team9432.auto.types.FourNote
import org.team9432.auto.types.FourNote.EndAction.*
import org.team9432.auto.types.FourNote.StartingPosition
import org.team9432.choreogenerator.Position
import org.team9432.choreogenerator.StraightLine
import org.team9432.lib.unit.degrees
import org.team9432.lib.unit.inches
import org.team9432.lib.unit.meters

object FourNotePaths {
    private const val AUTO_KEY = "4Note"

    private val FOUR_NOTE_START = Position(1.34.meters, AutoFieldConstants.speakerYCoordinate, 180.0.degrees)
    private val FOUR_NOTE_AMP_START = Position(0.75.meters, 6.63.meters, -120.degrees)
    private val FOUR_NOTE_STAGE_START = Position(0.75.meters, 4.47.meters, 120.degrees)

    private val FOUR_NOTE_SHOT = Position(2.0.meters, AutoFieldConstants.speakerYCoordinate, 180.0.degrees)
    private val FOUR_NOTE_AMP_ALIGN = AutoFieldConstants.ampNote.copy().moveX(-0.6.meters).pointAwayFrom(AutoFieldConstants.ampNote)
    private val FOUR_NOTE_AMP_PICKUP = AutoFieldConstants.ampNote.copy().moveX(-0.2.meters).pointAwayFrom(AutoFieldConstants.ampNote)
    private val FOUR_NOTE_AMP_EXIT = AutoFieldConstants.ampNote.copy().move(x = -0.5.meters, y = -0.6.meters, heading = 180.degrees)
    private val FOUR_NOTE_SPEAKER_PICKUP = Position(2.88.meters, AutoFieldConstants.speakerYCoordinate, 180.0.degrees)/*This part is because of a choreo bug, not to align better*/.moveX(-5.inches)
    private val FOUR_NOTE_STAGE_ALIGN = AutoFieldConstants.stageNote.copy().moveX(-0.6.meters).pointAwayFrom(AutoFieldConstants.stageNote)
    private val FOUR_NOTE_STAGE_PICKUP = AutoFieldConstants.stageNote.copy().moveX(-0.2.meters).pointAwayFrom(AutoFieldConstants.stageNote)
    private val FOUR_NOTE_STAGE_EXIT = AutoFieldConstants.stageNote.copy().move(x = -0.5.meters, y = 0.6.meters, heading = 180.degrees)

    private val FOUR_NOTE_ABOVE_STAGE_DRIVE = Position(5.5.meters, 6.3.meters, 180.0.degrees)
    private val FOUR_NOTE_THROUGH_STAGE_DRIVE_CLOSE = Position(4.4.meters, 5.meters, 180.0.degrees)
    private val FOUR_NOTE_THROUGH_STAGE_DRIVE_MIDDLE = Position(5.meters, 4.3.meters, 180.0.degrees)
    private val FOUR_NOTE_THROUGH_STAGE_DRIVE_FAR = Position(6.meters, 4.meters, 180.0.degrees)

    fun getSegmentsFor(config: FourNote): List<AutoSegment> {
        val fourNote = basicFourNote(config.startingPosition, config.ampFirst)

        return when (config.endAction) {
            DO_NOTHING -> fourNote
            SCORE_CENTERLINE -> fourNote + centerNote(config.centerNote)
            DRIVE_TO_CENTER -> fourNote + driveToCenterEnd()
        }
    }

    private fun driveToCenterEnd() = AutoSegment("${AUTO_KEY}DriveToCenterEnd") {
        addPoseWaypoint(FOUR_NOTE_SHOT)
        addTranslationWaypoint(getCenterNoteAlignPose(ONE))
    }

    private fun centerNote(note: CenterNote) = AutoSegment("${AUTO_KEY}Center${note.readableName}") {
        if (note !in FourNote.validCenterNotes) throw UnsupportedOperationException("Note ${note.name} is not supported!")
        addPoseWaypoint(FOUR_NOTE_SHOT)

        val travelPath: ((Boolean) -> Unit)? = when (note) {
            ONE -> null
            TWO -> { _ -> addTranslationWaypoint(FOUR_NOTE_ABOVE_STAGE_DRIVE) }
            THREE, FOUR -> { returning ->
                if (returning) {
                    addTranslationWaypoint(FOUR_NOTE_THROUGH_STAGE_DRIVE_FAR)
                    addTranslationWaypoint(FOUR_NOTE_THROUGH_STAGE_DRIVE_MIDDLE)
                    addTranslationWaypoint(FOUR_NOTE_THROUGH_STAGE_DRIVE_CLOSE)
                } else {
                    addTranslationWaypoint(FOUR_NOTE_THROUGH_STAGE_DRIVE_CLOSE)
                    addTranslationWaypoint(FOUR_NOTE_THROUGH_STAGE_DRIVE_MIDDLE)
                    addTranslationWaypoint(FOUR_NOTE_THROUGH_STAGE_DRIVE_FAR)
                }
            }

            else -> throw UnsupportedOperationException()
        }

        travelPath?.invoke(/*returning =*/false)

        val alignPose = addPoseWaypoint(getCenterNoteAlignPose(note))
        val intakePose = addPoseWaypoint(getCenterNoteIntakePose(note))
        addConstraint(StraightLine(alignPose, intakePose))

        travelPath?.invoke(/*returning =*/true)

        addPoseWaypoint(FOUR_NOTE_SHOT)
    }

    private fun preload(startingPosition: StartingPosition) = AutoSegment("4NotePreloadFrom${startingPosition.readableName}") {
        when (startingPosition) {
            StartingPosition.AMP -> {
                addPoseWaypoint(FOUR_NOTE_AMP_START)
                addTranslationWaypoint(FOUR_NOTE_AMP_START.copy().moveX(0.5.meters))
            }
            StartingPosition.STAGE -> {
                addPoseWaypoint(FOUR_NOTE_STAGE_START)
                addTranslationWaypoint(FOUR_NOTE_STAGE_START.copy().moveX(0.5.meters))
            }
            StartingPosition.CENTER -> addPoseWaypoint(FOUR_NOTE_START)
        }
        addPoseWaypoint(FOUR_NOTE_SHOT)
    }

    private fun basicFourNote(startingPosition: StartingPosition, ampFirst: Boolean): List<AutoSegment> {
        val noteList = if (ampFirst) listOf(AMP, SPEAKER, STAGE) else listOf(STAGE, SPEAKER, AMP)
        return listOf(preload(startingPosition)) + noteList.map { closeNote(it) }
    }

    private fun closeNote(note: AutoFieldConstants.CloseNote) = AutoSegment("${AUTO_KEY}Close${note.readableName}") {
        addPoseWaypoint(FOUR_NOTE_SHOT)

        when (note) {
            AMP -> {
                val align = addPoseWaypoint(FOUR_NOTE_AMP_ALIGN)
                val pickup = addPoseWaypoint(FOUR_NOTE_AMP_PICKUP)
                addConstraint(StraightLine(align, pickup))
                addTranslationWaypoint(FOUR_NOTE_AMP_EXIT)
                addPoseWaypoint(FOUR_NOTE_SHOT)
            }

            SPEAKER -> {
                addPoseWaypoint(FOUR_NOTE_SPEAKER_PICKUP)
                addPoseWaypoint(FOUR_NOTE_SHOT)
            }

            STAGE -> {
                val align = addPoseWaypoint(FOUR_NOTE_STAGE_ALIGN)
                val pickup = addPoseWaypoint(FOUR_NOTE_STAGE_PICKUP)
                addConstraint(StraightLine(align, pickup))
                addTranslationWaypoint(FOUR_NOTE_STAGE_EXIT)
                addPoseWaypoint(FOUR_NOTE_SHOT)
            }
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
}