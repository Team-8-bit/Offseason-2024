package org.team9432.resources.drive.controllers

import choreo.trajectory.SwerveSample
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.littletonrobotics.junction.Logger
import org.team9432.RobotPosition
import org.team9432.lib.dashboard.LoggedTunableNumber
import org.team9432.lib.unit.inMeters
import org.team9432.lib.util.distanceTo

class ChoreoTrajectoryController {
    private val xController = PIDController(translationkP, 0.0, translationkD)
    private val yController = PIDController(translationkP, 0.0, translationkD)
    private val rController = PIDController(rotationkP, 0.0, rotationkD).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }

    companion object {
        private const val TABLE_KEY = "ChoreoTrajectoryController"

        private val translationkP by LoggedTunableNumber("$TABLE_KEY/translationkP", 1.0)
        private val translationkD by LoggedTunableNumber("$TABLE_KEY/translationkD", 0.0)
        private val rotationkP by LoggedTunableNumber("$TABLE_KEY/rotationkP", 1.0)
        private val rotationkD by LoggedTunableNumber("$TABLE_KEY/rotationkD", 0.0)
    }

    fun calculate(targetPose: Pose2d, sample: SwerveSample): ChassisSpeeds {
        val currentPose = RobotPosition.currentPose

        val xFF = sample.vx
        val yFF = sample.vy
        val rotationFF = sample.omega

        val xFeedback = xController.calculate(currentPose.x, sample.x)
        val yFeedback = yController.calculate(currentPose.y, sample.y)
        val rotationFeedback = rController.calculate(currentPose.rotation.radians, sample.heading)

        val output = ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            rotationFF + rotationFeedback,
            currentPose.rotation
        )

        Logger.recordOutput("$TABLE_KEY/SetpointPose", sample.pose)
        Logger.recordOutput("$TABLE_KEY/SetpointSpeeds", sample.chassisSpeeds)
        Logger.recordOutput("$TABLE_KEY/OutputSpeeds", output)
        Logger.recordOutput("$TABLE_KEY/TranslationErrorMeters", currentPose.distanceTo(sample.pose).inMeters)
        Logger.recordOutput("$TABLE_KEY/RotationErrorDegrees", currentPose.rotation.minus(sample.pose.rotation).degrees)

        return output
    }
}