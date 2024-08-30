package org.team9432.auto.paths

import org.team9432.auto.paths.AutoFieldConstants.CenterNote
import org.team9432.auto.types.AmpsideCenterline
import org.team9432.auto.types.AutoSegment
import org.team9432.auto.types.StartingPosition
import org.team9432.choreogenerator.ChoreoTrajectory
import org.team9432.choreogenerator.MaxAngVelocity
import org.team9432.choreogenerator.Position
import org.team9432.choreogenerator.StraightLine
import org.team9432.lib.unit.degrees
import org.team9432.lib.unit.meters

object AmpsideCenterlinePaths {
    private const val AUTO_KEY = "AmpsideCenterline"

    private val SHOOTING_POSITION = Position(1.65.meters, 6.5.meters).pointAt(AutoFieldConstants.speakerAimPosition)

    private val AMP_ALIGN = AutoFieldConstants.ampNote.copy().moveX(-0.6.meters).pointAwayFrom(AutoFieldConstants.ampNote)
    private val AMP_PICKUP = AutoFieldConstants.ampNote.copy().moveX(-0.2.meters).pointAwayFrom(AutoFieldConstants.ampNote)

    private val AVOID_STAGE = Position(5.meters, 7.meters, SHOOTING_POSITION.heading)

    private val AMP_DODGE_CLOSE = Position(2.meters, 7.75.meters, 180.0.degrees)
    private val AMP_DODGE_FAR = Position(3.5.meters, 7.75.meters, 180.0.degrees)

    fun getSegmentsFor(config: AmpsideCenterline): List<AutoSegment> {
        val paths = mutableListOf<AutoSegment>()

        paths += preload(config.startingPosition)

        if (config.scoreCloseAmpNote) {
            paths += scoreAmpNote()
        }

        paths += config.notes.map { centerNote(it, dodgeAmpNote = !config.scoreCloseAmpNote) }

        return paths
    }

    private fun ChoreoTrajectory.ampNoteDodge(returning: Boolean) {
        if (returning) {
            val start = addPoseWaypoint(AMP_DODGE_FAR)
            val end = addPoseWaypoint(AMP_DODGE_CLOSE)
            addConstraint(StraightLine(start, end))
            addConstraint(MaxAngVelocity(start, end, 0.0))
        } else {
            val start = addPoseWaypoint(AMP_DODGE_CLOSE)
            val end = addPoseWaypoint(AMP_DODGE_FAR)
            addConstraint(StraightLine(start, end))
            addConstraint(MaxAngVelocity(start, end, 0.0))
        }
    }

    private fun centerNote(note: CenterNote, dodgeAmpNote: Boolean) = AutoSegment("${AUTO_KEY}Center${note.readableName}WithDodge$dodgeAmpNote") {
        if (note !in AmpsideCenterline.validCenterNotes) throw UnsupportedOperationException("Note ${note.name} is not supported!")
        addPoseWaypoint(SHOOTING_POSITION)

        if (dodgeAmpNote) ampNoteDodge(returning = false)
        else if (note == CenterNote.TWO) addTranslationWaypoint(AVOID_STAGE)

//        addPoseWaypoint(SharedPositions.getCenterNoteAlignPose(note))
        addPoseWaypoint(SharedPositions.getCenterNoteIntakePose(note))

        if (dodgeAmpNote) ampNoteDodge(returning = true)
        else if (note == CenterNote.TWO) addTranslationWaypoint(AVOID_STAGE)

        addPoseWaypoint(SHOOTING_POSITION)
    }

    private fun scoreAmpNote() = AutoSegment("${AUTO_KEY}ScoreAmpNote") {
        addPoseWaypoint(SHOOTING_POSITION)
        addPoseWaypoint(AMP_ALIGN)
        addPoseWaypoint(AMP_PICKUP)
        addPoseWaypoint(SHOOTING_POSITION)
    }

    private fun preload(startingPosition: StartingPosition) = AutoSegment("${AUTO_KEY}PreloadFrom${startingPosition.readableName}") {
        when (startingPosition) {
            StartingPosition.AMP -> addPoseWaypoint(SharedPositions.AMP_START_POSITION)

            StartingPosition.STAGE -> {
                addPoseWaypoint(SharedPositions.STAGE_START_POSITION)
                addTranslationWaypoint(SharedPositions.STAGE_START_POSITION.copy().moveX(0.5.meters))
            }

            StartingPosition.CENTER -> addPoseWaypoint(SharedPositions.CENTER_START_POSITION)
        }
        addPoseWaypoint(SHOOTING_POSITION)
    }
}