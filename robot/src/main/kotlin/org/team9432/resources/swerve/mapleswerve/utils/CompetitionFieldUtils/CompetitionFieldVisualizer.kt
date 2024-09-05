package org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.littletonrobotics.junction.Logger
import org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils.Objects.GamePieceOnFlyDisplay
import org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils.Objects.RobotOnFieldDisplay

/**
 * visualizes a competition field on dashboard & Advantage Scope
 * displays robots, opponent robots and game pieces on field
 * the source of these information should either be captured from a vision system during a real competition
 * or by the Maple Physics Simulation during a simulated competition
 */
class CompetitionFieldVisualizer(val mainRobot: RobotOnFieldDisplay) {
    interface ObjectOnFieldDisplay {
        val typeName: String
        val pose3d: Pose3d
    }

    interface Object2dOnFieldDisplay: ObjectOnFieldDisplay {
        val objectOnFieldPose2d: Pose2d
        override val typeName: String
        override val pose3d: Pose3d
            get() {
                return Pose3d(objectOnFieldPose2d)
            }
    }

    private val objectsOnFieldWithGivenType: MutableMap<String?, MutableSet<ObjectOnFieldDisplay>>
    private val gamePiecesOnFlyDisplayWithGivenType: MutableMap<String?, MutableSet<GamePieceOnFlyDisplay>>
    private val dashboardField2d: Field2d

    init {
        this.objectsOnFieldWithGivenType = HashMap()
        this.gamePiecesOnFlyDisplayWithGivenType = HashMap()
        dashboardField2d = Field2d()
        SmartDashboard.putData("Field", dashboardField2d)
    }

    fun addObject(`object`: ObjectOnFieldDisplay): ObjectOnFieldDisplay {
        if (!objectsOnFieldWithGivenType.containsKey(`object`.typeName)) objectsOnFieldWithGivenType.put(`object`.typeName, HashSet())
        objectsOnFieldWithGivenType.get(`object`.typeName)!!.add(`object`)
        return `object`
    }

    fun deleteObject(`object`: ObjectOnFieldDisplay): ObjectOnFieldDisplay? {
        if (!objectsOnFieldWithGivenType.containsKey(`object`.typeName)) return null
        if (objectsOnFieldWithGivenType.get(`object`.typeName)!!.remove(`object`)) return `object`
        return null
    }

    fun addGamePieceOnFly(gamePieceOnFlyDisplay: GamePieceOnFlyDisplay): GamePieceOnFlyDisplay {
        addObject(gamePieceOnFlyDisplay)
        if (!gamePiecesOnFlyDisplayWithGivenType.containsKey(gamePieceOnFlyDisplay.typeName)) gamePiecesOnFlyDisplayWithGivenType.put(gamePieceOnFlyDisplay.typeName, HashSet())
        gamePiecesOnFlyDisplayWithGivenType.get(gamePieceOnFlyDisplay.typeName)!!.add(gamePieceOnFlyDisplay)
        return gamePieceOnFlyDisplay
    }

    fun clearObjectsWithGivenType(typeName: String?): Set<ObjectOnFieldDisplay> {
        if (!objectsOnFieldWithGivenType.containsKey(typeName)) return HashSet()
        val originalSet: Set<ObjectOnFieldDisplay> = objectsOnFieldWithGivenType.get(typeName)!!
        objectsOnFieldWithGivenType.put(typeName, HashSet())
        return originalSet
    }

    fun displayTrajectory(trajectory: Array<Pose2d?>) {
        dashboardField2d.getObject("trajectory").setPoses(*trajectory)
    }

    fun updateObjectsToDashboardAndTelemetry() {
        removeGamePiecesOnFlyIfReachedTarget()
        for (typeName: String? in objectsOnFieldWithGivenType.keys) {
            val objects: Set<ObjectOnFieldDisplay> = objectsOnFieldWithGivenType.get(typeName)!!
            Logger.recordOutput("/Field/" + typeName, *getPose3ds(objects))
        }

        dashboardField2d.setRobotPose(mainRobot.objectOnFieldPose2d)
        Logger.recordOutput("/Field/Robot", mainRobot.objectOnFieldPose2d)
    }

    private fun removeGamePiecesOnFlyIfReachedTarget() {
        for (gamePieceSet: MutableSet<GamePieceOnFlyDisplay> in gamePiecesOnFlyDisplayWithGivenType.values) gamePieceSet.removeIf { gamePieceOnFlyDisplay: GamePieceOnFlyDisplay ->
            if (gamePieceOnFlyDisplay.isReached) deleteObject(gamePieceOnFlyDisplay)
            gamePieceOnFlyDisplay.isReached
        }
    }

    companion object {
        private fun getPose2ds(objects: Set<ObjectOnFieldDisplay>): List<Pose2d> {
            val pose2dList: MutableList<Pose2d> = ArrayList()

            for (`object`: ObjectOnFieldDisplay in objects) pose2dList.add(`object`.pose3d.toPose2d())
            return pose2dList
        }

        private fun getPose3ds(objects: Set<ObjectOnFieldDisplay>): Array<Pose3d> {
            return objects.map { obj: ObjectOnFieldDisplay -> obj.pose3d }.toTypedArray()
        }
    }
}
