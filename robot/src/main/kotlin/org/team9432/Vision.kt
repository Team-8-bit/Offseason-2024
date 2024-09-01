package org.team9432

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.constants.EvergreenFieldConstants.isOnField
import org.team9432.lib.doglog.Logger
import org.team9432.oi.Controls
import org.team9432.resources.swerve.Swerve
import kotlin.jvm.optionals.getOrNull
import kotlin.math.abs

// https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/swervedriveposeestsim/
object Vision {
    val isEnabled get() = !Controls.forceDisableVision && (camera.isConnected || Robot.isSimulated)

    private val camera = PhotonCamera("Limelight")
    private val photonPoseEstimator = PhotonPoseEstimator(FieldConstants.aprilTagFieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCameraArducam)

    init {
        photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)

        RobotPeriodicManager.startPeriodic {
            update()
        }
    }


    private fun update() {
        Logger.log("Vision/Connected", camera.isConnected)
        Logger.log("Vision/Enabled", isEnabled)

        val estimatorResult = photonPoseEstimator.update().getOrNull()

        photonPoseEstimator.setReferencePose(Swerve.getRobotPose())

        if (estimatorResult != null) {
            val pose = estimatorResult.estimatedPose

            if (pose.isValid()) {
                val pose2d = pose.toPose2d()
                Logger.log("Vision/EstimatedPose", pose)
                Swerve.addVisionMeasurement(pose2d, estimatorResult.timestampSeconds, getEstimationStdDevs(pose2d))
            }

            Logger.log("Vision/StrategyUsed", estimatorResult.strategy.name)


            val targetsUsed = estimatorResult.targetsUsed
            Logger.log("Vision/TrackedTags", targetsUsed.map { it.fiducialId }.toIntArray())
            targetsUsed.forEach { target ->
                val baseKey = "Vision/Targets/${target.fiducialId}"
                Logger.log("$baseKey/PoseAmbiguity", target.poseAmbiguity)
                Logger.log("$baseKey/Area", target.area)
            }
        }
    }

    private val singleTagStdDevs: Matrix<N3, N1> = VecBuilder.fill(0.25, 0.25, 999.0)
    private val multiTagStdDevs: Matrix<N3, N1> = VecBuilder.fill(0.01, 0.01, 3.0)

    private fun getEstimationStdDevs(estimatedPose: Pose2d): Matrix<N3, N1> {
        var estStdDevs = singleTagStdDevs
        val targets = camera.latestResult.getTargets()
        var numTags = 0
        var avgDist = 0.0

        for (target in targets) {
            val tagPose = photonPoseEstimator.fieldTags.getTagPose(target.fiducialId)
            if (tagPose.isEmpty) continue
            numTags++
            avgDist += tagPose.get().toPose2d().translation.getDistance(estimatedPose.translation)
        }

        if (numTags == 0) return estStdDevs
        avgDist /= numTags.toDouble()

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = multiTagStdDevs

        // Increase std devs based on (average) distance
        estStdDevs = if (numTags == 1 && avgDist > 4) VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
        else estStdDevs.times(1 + (avgDist * avgDist / 30))

        return estStdDevs
    }

    /** Check that the given position is close to the floor and within the field walls. */
    private fun Pose3d.isValid() = abs(z) < 0.25 && this.toPose2d().isOnField()

    private val robotToCameraArducam
        get() = Transform3d(
            Translation3d(
                Units.inchesToMeters(5.875),
                -Units.inchesToMeters(8.875),
                Units.inchesToMeters(8.5)
            ),
            Rotation3d(
                0.0,
                -Math.toRadians(28.125000),
                Math.toRadians(6.920208)
            )
        )
}