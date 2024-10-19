package org.team9432

import org.team9432.lib.unit.Translation2d
import org.team9432.lib.unit.meters
import org.team9432.lib.util.applyFlip

object PositionConstants {
    /** Position the robot aims at when shooting in the speaker. */
    val speakerAimPose
        get() = Translation2d(0.22.meters, FieldConstants.speakerYCoordinate).applyFlip()
}