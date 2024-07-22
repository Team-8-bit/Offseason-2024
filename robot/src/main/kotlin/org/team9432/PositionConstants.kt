package org.team9432

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import org.team9432.lib.constants.EvergreenFieldConstants
import org.team9432.lib.unit.Translation2d
import org.team9432.lib.unit.meters
import org.team9432.resources.swerve.Swerve

object PositionConstants {
    /** Position the robot aims at when shooting in the speaker. */
    val speakerAimPose = Translation2d(0.35.meters, FieldConstants.speakerYCoordinate)

    /** Position the robot aims when feeding, it is in the amp corner of the field. */
    val feedAimPose = Translation2d(0.0.meters, EvergreenFieldConstants.lengthY)

    /** Position the robot should feed notes from. */
    val feedPose = Translation2d(EvergreenFieldConstants.centerX + 1.0.meters, 1.0.meters).angleAtFeedCorner()

    /** Returns a [Pose2d] using this [Translation2d]'s coordinates, rotated to point at the corner where notes should be fed to. */
    private fun Translation2d.angleAtFeedCorner() = Pose2d(x, y, Swerve.angleTo(feedAimPose, currentPose = Translation2d(x, y)))
}