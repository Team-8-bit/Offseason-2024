package org.team9432.auto.paths

import org.team9432.auto.paths.AutoFieldConstants.CenterNote
import org.team9432.auto.paths.AutoFieldConstants.CenterNote.*
import org.team9432.auto.paths.AutoFieldConstants.CloseNote.*
import org.team9432.auto.types.AutoType
import org.team9432.choreogenerator.ChoreoTrajectory
import org.team9432.choreogenerator.MaxAngVelocity
import org.team9432.choreogenerator.Position
import org.team9432.choreogenerator.StraightLine
import org.team9432.lib.unit.degrees
import org.team9432.lib.unit.inches
import org.team9432.lib.unit.meters

object FourNotePaths {
    private const val AUTO_KEY = "4Note"

    private val FOUR_NOTE_START = Position(1.34.meters, AutoFieldConstants.speakerYCoordinate, 180.0.degrees)
    private val FOUR_NOTE_SHOT = Position(1.7.meters, AutoFieldConstants.speakerYCoordinate, 180.0.degrees)
    private val FOUR_NOTE_AMP_ALIGN = AutoFieldConstants.ampNote.copy().moveX(-0.6.meters).pointAwayFrom(AutoFieldConstants.ampNote)
    private val FOUR_NOTE_AMP_PICKUP = AutoFieldConstants.ampNote.copy().moveX(-0.2.meters).pointAwayFrom(AutoFieldConstants.ampNote)
    private val FOUR_NOTE_AMP_EXIT = AutoFieldConstants.ampNote.copy().move(x = -0.5.meters, y = -0.5.meters, heading = 180.degrees)
    private val FOUR_NOTE_SPEAKER_PICKUP = Position(2.88.meters, AutoFieldConstants.speakerYCoordinate, 180.0.degrees)/*This part is because of a choreo bug, not to align better*/.moveX(-3.inches)
    private val FOUR_NOTE_STAGE_ALIGN = AutoFieldConstants.stageNote.copy().moveX(-0.6.meters).pointAwayFrom(AutoFieldConstants.stageNote)
    private val FOUR_NOTE_STAGE_PICKUP = AutoFieldConstants.stageNote.copy().moveX(-0.2.meters).pointAwayFrom(AutoFieldConstants.stageNote)
    private val FOUR_NOTE_STAGE_EXIT = AutoFieldConstants.stageNote.copy().move(x = -0.5.meters, y = 0.5.meters, heading = 180.degrees)

    private val FOUR_NOTE_ABOVE_STAGE_DRIVE = Position(5.5.meters, 6.3.meters, 180.0.degrees)
    private val FOUR_NOTE_THROUGH_STAGE_DRIVE_CLOSE = Position(4.4.meters, 5.meters, 180.0.degrees)
    private val FOUR_NOTE_THROUGH_STAGE_DRIVE_MIDDLE = Position(5.meters, 4.3.meters, 180.0.degrees)
    private val FOUR_NOTE_THROUGH_STAGE_DRIVE_FAR = Position(6.meters, 4.meters, 180.0.degrees)

    private fun getCenterNoteAlignPose(note: CenterNote): Position {
        val notePose = AutoFieldConstants.getNotePose(note)
        return notePose.copy().moveX(-.6.meters).pointAwayFrom(notePose)
    }

    private fun getCenterNoteIntakePose(note: CenterNote): Position {
        val notePose = AutoFieldConstants.getNotePose(note)
        return notePose.copy().moveX(-.2.meters).pointAwayFrom(notePose)
    }

    fun generate(config: AutoType.FourNote): List<ChoreoTrajectory> {
        val fourNote = basicFourNote(config.ampFirst)

        val centerNote = config.centerNote?.let { centerNote(it) }

        if (centerNote == null) return fourNote
        else return fourNote + centerNote
    }

    private fun centerNote(note: CenterNote) = ChoreoTrajectory.new("${AUTO_KEY}Center${note.readableName}") {
        if (note !in AutoType.FourNote.validCenterNotes) throw UnsupportedOperationException("Note ${note.name} is not supported!")
        addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)

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

        addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)
    }

    private fun preload() = ChoreoTrajectory.new("${AUTO_KEY}Preload") {
        addPoseWaypoint(FOUR_NOTE_START)
        addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)
    }

    private fun basicFourNote(ampFirst: Boolean): List<ChoreoTrajectory> {
        val noteList = if (ampFirst) listOf(AMP, SPEAKER, STAGE) else listOf(STAGE, SPEAKER, AMP)

        return listOf(preload()) + noteList.map { closeNote(it) }
    }

    private fun closeNote(note: AutoFieldConstants.CloseNote) = ChoreoTrajectory.new("${AUTO_KEY}Close${note.readableName}") {
        addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)

        when (note) {
            AMP -> {
                val align = addPoseWaypoint(FOUR_NOTE_AMP_ALIGN)
                val pickup = addPoseWaypoint(FOUR_NOTE_AMP_PICKUP)
                addConstraint(StraightLine(align, pickup))
                addTranslationWaypoint(FOUR_NOTE_AMP_EXIT)
                addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)
            }

            SPEAKER -> {
                addPoseWaypoint(FOUR_NOTE_SPEAKER_PICKUP)
                addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)
            }

            STAGE -> {
                val align = addPoseWaypoint(FOUR_NOTE_STAGE_ALIGN)
                val pickup = addPoseWaypoint(FOUR_NOTE_STAGE_PICKUP)
                addConstraint(StraightLine(align, pickup))
                addTranslationWaypoint(FOUR_NOTE_STAGE_EXIT)
                addPoseWaypoint(FOUR_NOTE_SHOT, stopPoint = true)
            }
        }
    }
}