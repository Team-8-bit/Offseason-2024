package org.team9432.resources.swerve

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import org.team9432.lib.util.allianceSwitch
import java.util.function.Supplier

object FieldConstants {
    const val FIELD_WIDTH: Double = 16.54
    const val FIELD_HEIGHT: Double = 8.21

    @JvmField
    val SPEAKER_POSE_BLUE: Translation3d = Translation3d(0.1, 5.55, 2.2)

    val SPEAKER_POSITION_SUPPLIER: Supplier<Translation2d> = Supplier { toCurrentAllianceTranslation(SPEAKER_POSE_BLUE.toTranslation2d()) }

    fun toCurrentAllianceRotation(rotationAtBlueSide: Rotation2d): Rotation2d {
        val yAxis = Rotation2d.fromDegrees(90.0)
        val differenceFromYAxisAtBlueSide = rotationAtBlueSide.minus(yAxis)
        val differenceFromYAxisNew = differenceFromYAxisAtBlueSide.times((if (isSidePresentedAsRed) -1 else 1).toDouble())
        return yAxis.rotateBy(differenceFromYAxisNew)
    }

    fun toCurrentAllianceTranslation(translationAtBlueSide: Translation2d): Translation2d {
        if (isSidePresentedAsRed) return Translation2d(
            FIELD_WIDTH - translationAtBlueSide.x,
            translationAtBlueSide.y
        )
        return translationAtBlueSide
    }

    fun toCurrentAllianceTranslation(translation3dAtBlueSide: Translation3d): Translation3d {
        val translation3dAtCurrentAlliance = toCurrentAllianceTranslation(translation3dAtBlueSide.toTranslation2d())
        if (isSidePresentedAsRed) return Translation3d(
            translation3dAtCurrentAlliance.x,
            translation3dAtCurrentAlliance.y,
            translation3dAtBlueSide.z
        )
        return translation3dAtBlueSide
    }

    fun toCurrentAlliancePose(poseAtBlueSide: Pose2d): Pose2d {
        return Pose2d(
            toCurrentAllianceTranslation(poseAtBlueSide.translation),
            toCurrentAllianceRotation(poseAtBlueSide.rotation)
        )
    }

    val isSidePresentedAsRed: Boolean
        get() {
            val alliance = DriverStation.getAlliance()
            return alliance.isPresent && alliance.get() == Alliance.Red
        }

    val driverStationFacing: Rotation2d
        get() = allianceSwitch(red = Rotation2d(Math.PI), blue = Rotation2d(0.0))
}