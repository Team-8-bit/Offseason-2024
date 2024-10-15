package org.team9432.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.littletonrobotics.junction.Logger
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult
import org.team9432.FieldConstants.apriltagFieldLayout
import org.team9432.RobotState
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.constants.EvergreenFieldConstants.isOnField
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs

class Vision(private val io: VisionIO) {
    private val inputs = LoggedVisionIOInputs()

    private val poseEstimator = PhotonPoseEstimator(
        apriltagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        VisionConstants.robotToCamera
    )

    init {
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
        RobotPeriodicManager.startPeriodic { periodic() }
    }

    private fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Vision", inputs)

        Logger.recordOutput("Vision/Connected", inputs.isConnected)
        Logger.recordOutput("Vision/TrackedTagIds", *inputs.results.targets.mapNotNull { apriltagFieldLayout.getTagPose(it.fiducialId).getOrNull() }.toTypedArray())

        if (inputs.isConnected) applyToPoseEstimator(inputs.results)
    }

    private fun applyToPoseEstimator(result: PhotonPipelineResult) {
        // Get the estimated position or return
        val estimatedRobotPose = poseEstimator.update(result).getOrNull() ?: return

        // Make sure the estimation is valid, i.e. not in the floor or outside the field
        if (!estimatedRobotPose.estimatedPose.isValid()) return

        val visionPose = estimatedRobotPose.estimatedPose.toPose2d()
        val timestamp = estimatedRobotPose.timestampSeconds
        val stdDevs: Matrix<N3, N1> = getEstimationStdDevs(estimatedRobotPose)

        RobotState.applyVisionMeasurement(visionPose, timestamp, stdDevs)
    }

    // Just stolen from 8033
    val visionPointBlankDevs: Matrix<N3, N1> = VecBuilder.fill(1.4, 1.4, 5.0)
    val distanceFactor = 0.5

    private fun getEstimationStdDevs(estimatedPose: EstimatedRobotPose): Matrix<N3, N1> {
        // A count of the number of valid tags seen
        var numTags = 0

        // Get the average distance of apriltags that the camera sees
        val avgDist = estimatedPose.targetsUsed.map { target ->
            val tagPose = apriltagFieldLayout.getTagPose(target.fiducialId)
            if (tagPose.isEmpty) return@map null
            numTags++

            val estimatedPose2d = estimatedPose.estimatedPose.toPose2d()
            tagPose.get().toPose2d().translation.getDistance(estimatedPose2d.translation)
        }.filterNotNull().average()

        Logger.recordOutput("Vision/TagAvgDistance", avgDist)


        var deviation = visionPointBlankDevs.times(avgDist * distanceFactor);
        if (estimatedPose.targetsUsed.size == 1) {
            deviation = deviation.times(2.0)
        }

        return deviation
//        when {
//            numTags == 0 -> return VisionConstants.singleTagStdDevs
//            numTags == 1 -> {
//                if (avgDist > 5) VisionConstants.maxStandardDeviations
//                else VisionConstants.singleTagStdDevs.times(1 + (avgDist * avgDist) / 30)
//            }
//
//            numTags > 1 -> {
//                if (avgDist > 7.0) VisionConstants.maxStandardDeviations
//                else VisionConstants.multiTagStdDevs
//            }
//
//            else -> VisionConstants.maxStandardDeviations
//        }
    }

    val isConnected get() = inputs.isConnected

    /** Check that the given position is close to the floor and within the field walls. */
    private fun Pose3d.isValid() = abs(z) < 0.25 && this.isOnField()
}