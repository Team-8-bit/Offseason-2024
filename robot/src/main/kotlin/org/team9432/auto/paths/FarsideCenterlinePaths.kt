package org.team9432.auto.paths

import org.team9432.auto.paths.AutoFieldConstants.CenterNote
import org.team9432.auto.paths.AutoFieldConstants.CenterNote.FIVE
import org.team9432.auto.paths.AutoFieldConstants.CenterNote.FOUR
import org.team9432.auto.types.AutoSegment
import org.team9432.auto.types.FarsideCenterline
import org.team9432.auto.types.StartingPosition
import org.team9432.choreogenerator.Position
import org.team9432.choreogenerator.StraightLine
import org.team9432.lib.unit.degrees
import org.team9432.lib.unit.meters

object FarsideCenterlinePaths {
    private const val AUTO_KEY = "FarsideCenterline"

    private val SHOOTING_POSITION = Position(1.65.meters, 4.54.meters).pointAt(AutoFieldConstants.speakerAimPosition)

    private val DRIVE_UNDER = Position(3.75.meters, 1.5.meters, 180.degrees)

    fun getSegmentsFor(config: FarsideCenterline): List<AutoSegment> {
        val preload = preload(config.startingPosition)
        val centerNotes = config.notes.map { centerNote(it) }

        return emptyList<AutoSegment>() + preload + centerNotes
    }

    private fun centerNote(note: CenterNote) = AutoSegment("${AUTO_KEY}Center${note.readableName}") {
        if (note !in FarsideCenterline.validCenterNotes) throw UnsupportedOperationException("Note ${note.name} is not supported!")
        addPoseWaypoint(SHOOTING_POSITION)

        val travelPath: ((Boolean) -> Unit) = when (note) {
            FOUR, FIVE -> { _ ->
                addTranslationWaypoint(DRIVE_UNDER)
            }

            else -> throw UnsupportedOperationException()
        }

        travelPath.invoke(/*returning =*/false)

        val alignPose = addPoseWaypoint(SharedPositions.getCenterNoteAlignPose(note))
        val intakePose = addPoseWaypoint(SharedPositions.getCenterNoteIntakePose(note))
        addConstraint(StraightLine(alignPose, intakePose))

        travelPath.invoke(/*returning =*/true)

        addPoseWaypoint(SHOOTING_POSITION)
    }

    private fun preload(startingPosition: StartingPosition) = AutoSegment("${AUTO_KEY}PreloadFrom${startingPosition.readableName}") {
        when (startingPosition) {
            StartingPosition.AMP -> {
                addPoseWaypoint(SharedPositions.AMP_START_POSITION)
                addTranslationWaypoint(SharedPositions.AMP_START_POSITION.copy().moveX(0.5.meters))
            }

            StartingPosition.STAGE -> addPoseWaypoint(SharedPositions.STAGE_START_POSITION)

            StartingPosition.CENTER -> addPoseWaypoint(SharedPositions.CENTER_START_POSITION)
        }
        addPoseWaypoint(SHOOTING_POSITION)
    }
}