package org.team9432.auto.paths

import org.team9432.auto.paths.AutoFieldConstants.CenterNote
import org.team9432.choreogenerator.Position
import org.team9432.lib.unit.degrees
import org.team9432.lib.unit.meters

object SharedPositions {
    val CENTER_START_POSITION = Position(1.34.meters, AutoFieldConstants.speakerYCoordinate, 180.0.degrees)
    val AMP_START_POSITION = Position(0.75.meters, 6.63.meters, -120.degrees)
    val STAGE_START_POSITION = Position(0.75.meters, 4.47.meters, 120.degrees)

    fun getCenterNoteAlignPose(note: CenterNote): Position {
        val notePose = AutoFieldConstants.getNotePose(note)
        return notePose.copy().moveX(-.6.meters).pointAwayFrom(notePose)
    }

    fun getCenterNoteIntakePose(note: CenterNote): Position {
        val notePose = AutoFieldConstants.getNotePose(note)
        return notePose.copy().moveX(-.2.meters).pointAwayFrom(notePose)
    }
}