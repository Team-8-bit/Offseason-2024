package org.team9432.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import org.photonvision.PhotonCamera
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import org.team9432.FieldConstants.apriltagFieldLayout
import org.team9432.lib.RobotPeriodicManager

class VisionIOSim(actualRobotPoseSupplier: () -> Pose2d): VisionIO {
    private val visionSim = VisionSystemSim("main")

    private val cameraProperties = SimCameraProperties().apply {
        setCalibration(1280, 800, Rotation2d.fromDegrees(78.61))
        setCalibError(0.25, 0.10)
//        exposureTimeMs = ?
        fps = 30.0
        avgLatencyMs = 25.0
        latencyStdDevMs = 8.0
    }

    private val camera = PhotonCamera(VisionConstants.CAMERA_NAME)
    private val cameraSim = PhotonCameraSim(camera, cameraProperties)

    init {
        visionSim.addAprilTags(apriltagFieldLayout)
        visionSim.addCamera(cameraSim, VisionConstants.robotToCamera)

        cameraSim.enableDrawWireframe(true)

        RobotPeriodicManager.startPeriodic { visionSim.update(actualRobotPoseSupplier.invoke()) }
    }

    override fun updateInputs(inputs: VisionIO.VisionIOInputs) {
        inputs.results = camera.latestResult
        inputs.isConnected = camera.isConnected
    }
}