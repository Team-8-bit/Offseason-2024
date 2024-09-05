package org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils.Simulations

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import org.team9432.resources.swerve.FieldConstants.FIELD_WIDTH
import org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils.Objects.Crescendo2024FieldObjects.NoteOnFieldSimulated

/**
 * field simulation for 2024 competition
 */
class Crescendo2024FieldSimulation(robot: HolonomicChassisSimulation): CompetitionFieldSimulation(robot, CrescendoFieldObstaclesMap()) {
    override fun placeGamePiecesOnField() {
        for (notePosition: Translation2d in NOTE_INITIAL_POSITIONS) super.addGamePiece(NoteOnFieldSimulated(notePosition))
    }

    /**
     * the obstacles on the 2024 competition field
     */
    class CrescendoFieldObstaclesMap: FieldObstaclesMap() {
        init {
            //left wall
            super.addBorderLine(
                Translation2d(0.0, 1.0),
                Translation2d(0.0, 4.51)
            )
            super.addBorderLine(
                Translation2d(0.0, 4.51),
                Translation2d(0.9, 5.0)
            )

            super.addBorderLine(
                Translation2d(0.9, 5.0),
                Translation2d(0.9, 6.05)
            )

            super.addBorderLine(
                Translation2d(0.9, 6.05),
                Translation2d(0.0, 6.5)
            )
            super.addBorderLine(
                Translation2d(0.0, 6.5),
                Translation2d(0.0, 8.2)
            )


            // upper wall
            super.addBorderLine(
                Translation2d(0.0, 8.12),
                Translation2d(FIELD_WIDTH, 8.12)
            )


            // righter wall 
            super.addBorderLine(
                Translation2d(FIELD_WIDTH, 1.0),
                Translation2d(FIELD_WIDTH, 4.51)
            )
            super.addBorderLine(
                Translation2d(FIELD_WIDTH, 4.51),
                Translation2d(FIELD_WIDTH - 0.9, 5.0)
            )
            super.addBorderLine(
                Translation2d(FIELD_WIDTH - 0.9, 5.0),
                Translation2d(FIELD_WIDTH - 0.9, 6.05)
            )
            super.addBorderLine(
                Translation2d(FIELD_WIDTH - 0.9, 6.05),
                Translation2d(FIELD_WIDTH, 6.5)
            )
            super.addBorderLine(
                Translation2d(FIELD_WIDTH, 6.5),
                Translation2d(FIELD_WIDTH, 8.2)
            )

            // lower wall
            super.addBorderLine(
                Translation2d(1.92, 0.0),
                Translation2d(FIELD_WIDTH - 1.92, 0.0)
            )

            // red source wall
            super.addBorderLine(
                Translation2d(1.92, 0.0),
                Translation2d(0.0, 1.0)
            )

            // blue source wall
            super.addBorderLine(
                Translation2d(FIELD_WIDTH - 1.92, 0.0),
                Translation2d(FIELD_WIDTH, 1.0)
            )

            // blue state
            super.addRectangularObstacle(
                0.35, 0.35,
                Pose2d(3.4, 4.1, Rotation2d())
            )
            super.addRectangularObstacle(
                0.35, 0.35,
                Pose2d(5.62, 4.1 - 1.28, Rotation2d.fromDegrees(30.0))
            )
            super.addRectangularObstacle(
                0.35, 0.35,
                Pose2d(5.62, 4.1 + 1.28, Rotation2d.fromDegrees(60.0))
            )

            // red stage
            super.addRectangularObstacle(
                0.35, 0.35,
                Pose2d(FIELD_WIDTH - 3.4, 4.1, Rotation2d())
            )
            super.addRectangularObstacle(
                0.35, 0.35,
                Pose2d(FIELD_WIDTH - 5.62, 4.1 - 1.28, Rotation2d.fromDegrees(60.0))
            )
            super.addRectangularObstacle(
                0.35, 0.35,
                Pose2d(FIELD_WIDTH - 5.62, 4.1 + 1.28, Rotation2d.fromDegrees(30.0))
            )
        }
    }

    companion object {
        private val NOTE_INITIAL_POSITIONS: Array<Translation2d> = arrayOf(
            Translation2d(2.9, 4.1),
            Translation2d(2.9, 5.55),
            Translation2d(2.9, 7.0),

            Translation2d(8.27, 0.75),
            Translation2d(8.27, 2.43),
            Translation2d(8.27, 4.1),
            Translation2d(8.27, 5.78),
            Translation2d(8.27, 7.46),

            Translation2d(13.64, 4.1),
            Translation2d(13.64, 5.55),
            Translation2d(13.64, 7.0),
        )
    }
}
