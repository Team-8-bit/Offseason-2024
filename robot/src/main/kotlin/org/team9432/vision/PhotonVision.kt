package org.team9432.vision

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArrayEntry
import edu.wpi.first.networktables.StructArrayPublisher
import edu.wpi.first.networktables.StructPublisher
import org.photonvision.PhotonCamera
import org.photonvision.targeting.PhotonPipelineResult
import org.team9432.FieldConstants
import org.team9432.lib.constants.EvergreenFieldConstants.isOnField
import org.team9432.lib.robot.CoroutineRobot
import org.team9432.lib.unit.*
import org.team9432.lib.util.set
import org.team9432.resources.swerve.Swerve
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs
import kotlin.math.floor

object PhotonVision {
    private val table = NetworkTableInstance.getDefault().getTable("Vision")
    private val camera = PhotonCamera("Limelight")
    private val robotToCamera = robotToCameraArducam

    private const val USE_MULTITAG = true

    init {
        CoroutineRobot.startPeriodic {
            update()
        }
    }

    private val connected = table.getBooleanTopic("Connected").publish()
    private val estimatedPose = table.getStructArrayTopic("Pose", Pose3d.struct).publish()
    private val trackedTags = table.getStructArrayTopic("TrackedTags", Pose3d.struct).publish()

    private fun update() {
        connected.set(camera.isConnected)

        val result = camera.latestResult

        // Record information
        result.targets.forEach { target -> table["Tags/${target.fiducialId}/Area"] = target.area }
        result.targets.groupBy { it.fiducialId }.forEach { (id, targets) ->
            targets.forEachIndexed { index, visionPose ->
                table["Tags/$id/Ambiguity-$index"] = visionPose.poseAmbiguity
            }
        }

        // Get the vision output
        val output = getVisionOutput(result)

        // Update the robot pose if the vision output isn't null
        if (output != null) {
            val (xyDeviation, pose, tagsUsed) = output
            Swerve.swerve.setVisionMeasurementStdDevs(VecBuilder.fill(xyDeviation.inMeters, xyDeviation.inMeters, 20.0.degrees.inDegrees))
            Swerve.swerve.addVisionMeasurement(pose.toPose2d(), result.timestampSeconds)
            trackedTags.set(tagsUsed.mapNotNull { FieldConstants.aprilTagFieldLayout.getTagPose(it).getOrNull() }.toTypedArray())
            estimatedPose.set(arrayOf(pose))
        } else {
            trackedTags.set(emptyArray<Pose3d>())
            estimatedPose.set(emptyArray<Pose3d>())
        }
    }

    private fun getVisionOutput(result: PhotonPipelineResult): VisionOutput? {
        if (!result.hasTargets()) return null // If the robot can't see any tags, don't bother

        val (pose, tagArea, tagsUsed) = if (!USE_MULTITAG) {
            updateNonMultitag(result)
        } else {
            // Attempt to run multitag, run non-multitag if it fails
            updateMultitag(result) ?: updateNonMultitag(result)
        } ?: return null

        val speed = Swerve.currentState.speeds

        // If the robot isn't really moving, and the tag is close, trust it a lot
        val xyDeviation = if (speed.vxMetersPerSecond + speed.vyMetersPerSecond <= 0.2 && tagArea > 0.3) {
            0.05.meters
        } else if (tagArea < 0.15) {
            2.0.meters
        } else if (tagArea < 0.4) {
            1.0.meters
        } else {
            return null // The tag is really far away
        }

        return VisionOutput(xyDeviation, pose, tagsUsed)
    }

    private fun updateMultitag(result: PhotonPipelineResult): VisionResult? {
        // Get the multitag result if possible
        val multiTagResult = result.multiTagResult
        if (!multiTagResult.estimatedPose.isPresent) return null

        // Calculate the robot position
        val cameraToField = result.multiTagResult.estimatedPose.best
        val pose = Pose3d().plus(cameraToField).relativeTo(FieldConstants.aprilTagFieldLayout.origin).plus(robotToCamera.inverse())

        // Check position validity
        if (!isPositionValid(pose)) return null

        // Return the vision result if everything worked
        val tagsUsed = result.getTargets().filter { multiTagResult.fiducialIDsUsed.contains(it.fiducialId) }
        val largestArea = tagsUsed.maxBy { it.area }.area

        return VisionResult(pose, largestArea, tagsUsed.map { it.fiducialId })
    }

    private fun updateNonMultitag(result: PhotonPipelineResult): VisionResult? {
        // Convert everything into visiontargets
        val poses = mutableListOf<VisionTarget>()
        for (target in result.targets) {
            val targetFiducialId = target.fiducialId
            val targetPosition = FieldConstants.aprilTagFieldLayout.getTagPose(targetFiducialId).getOrNull() ?: continue
            val estimatedPose = targetPosition.transformBy(target.bestCameraToTarget.inverse()).transformBy(robotToCamera.inverse())
            poses.add(VisionTarget(targetFiducialId, estimatedPose, target.poseAmbiguity, target.area))
        }

        // Filter out any bad estimations
        val filteredTargets = poses.filter {
            isPositionValid(it.pose) && it.ambiguity < 0.1
        }

        // Take the position closest to the current robot position from each tag
        val finalTargets = mutableListOf<VisionTarget>()
        filteredTargets.groupBy { it.id }.values.forEach { tagPoses -> finalTargets.add(tagPoses.minBy { Swerve.distanceTo(it.pose.toPose2d().translation).inMeters }) }

        // Record these final positions
//        Logger.recordOutput("Vision/AllPoses", *finalTargets.map { it.pose }.toTypedArray())

        // Return the one that's closest to where the robot already is
        val target = finalTargets.minByOrNull { Swerve.distanceTo(it.pose.toPose2d().translation).inMeters } ?: return null

        return VisionResult(target.pose, target.area, listOf(target.id))
    }

    /** Check that the given position is close to the floor and within the field walls. */
    private fun isPositionValid(pose: Pose3d) = abs(pose.z) < 0.25 && pose.toPose2d().isOnField()

    data class VisionTarget(val id: Int, val pose: Pose3d, val ambiguity: Double, val area: Double)
    data class VisionResult(val pose: Pose3d, val area: Double, val usedTags: List<Int>)
    data class VisionOutput(val visionStandardDeviation: Length, val pose: Pose3d, val tagsUsed: List<Int>)

    private val robotToCameraArducam
        get() = Transform3d(
            Translation3d(
                Units.inchesToMeters(7.301041),
                Units.inchesToMeters(-1.43244),
                Units.inchesToMeters(11.91629)
            ),
            Rotation3d(
                0.0,
                Math.toRadians(-12.0),
                0.0
            )
        )
}