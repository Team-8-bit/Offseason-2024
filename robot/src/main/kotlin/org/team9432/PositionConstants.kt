package org.team9432

import org.team9432.lib.constants.EvergreenFieldConstants
import org.team9432.lib.unit.Translation2d
import org.team9432.lib.unit.meters
import org.team9432.lib.util.applyFlip
import org.team9432.lib.util.pointAt

object PositionConstants {
    /** Position the robot aims at when shooting in the speaker. */
    val speakerAimPose
        get() = Translation2d(0.22.meters, FieldConstants.speakerYCoordinate).applyFlip()

    /** Position the robot aims when feeding, it is in the amp corner of the field. */
    val feedAimPose
        get() = Translation2d(0.0.meters, EvergreenFieldConstants.lengthY - 0.25.meters).applyFlip()
}