package org.team9432.resources.drive.controllers

import com.choreo.lib.ChoreoTrajectory
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.Logger
import org.team9432.Robot
import org.team9432.RobotPosition
import org.team9432.lib.dashboard.LoggedTunableNumber
import org.team9432.lib.unit.inMeters
import org.team9432.lib.util.applyFlip
import org.team9432.lib.util.distanceTo

class ChoreoTrajectoryController(private val trajectory: ChoreoTrajectory): GenericDriveController<ChassisSpeeds>() {
    private val xController = PIDController(translationkP, 0.0, translationkD)
    private val yController = PIDController(translationkP, 0.0, translationkD)
    private val rController = PIDController(rotationkP, 0.0, rotationkD).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }

    private val timer = Timer()

    companion object {
        private const val TABLE_KEY = "ChoreoTrajectoryController"

        private val translationkP by LoggedTunableNumber("$TABLE_KEY/translationkP", 1.0)
        private val translationkD by LoggedTunableNumber("$TABLE_KEY/translationkD", 0.0)
        private val rotationkP by LoggedTunableNumber("$TABLE_KEY/rotationkP", 1.0)
        private val rotationkD by LoggedTunableNumber("$TABLE_KEY/rotationkD", 0.0)
    }

    init {
        Logger.recordOutput("Trajectory/Poses", *trajectory.poses.map { it.applyFlip() }.toTypedArray())
        timer.start()
    }

    override fun calculate(): ChassisSpeeds {
        val currentPose = RobotPosition.currentPose

        val targetState = trajectory.sample(timer.get(), Robot.alliance == Alliance.Red)

        val xFF = targetState.velocityX
        val yFF = targetState.velocityY
        val rotationFF = targetState.angularVelocity

        val xFeedback = xController.calculate(currentPose.x, targetState.x)
        val yFeedback = yController.calculate(currentPose.y, targetState.y)
        val rotationFeedback = rController.calculate(currentPose.rotation.radians, targetState.heading)

        val output = ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            rotationFF + rotationFeedback,
            currentPose.rotation
        )

        Logger.recordOutput("$TABLE_KEY/SetpointPose", targetState.pose)
        Logger.recordOutput("$TABLE_KEY/SetpointSpeeds", targetState.chassisSpeeds)
        Logger.recordOutput("$TABLE_KEY/OutputSpeeds", output)
        Logger.recordOutput("$TABLE_KEY/TranslationErrorMeters", currentPose.distanceTo(targetState.pose).inMeters)
        Logger.recordOutput("$TABLE_KEY/RotationErrorDegrees", currentPose.rotation.minus(targetState.pose.rotation).degrees)
        Logger.recordOutput("$TABLE_KEY/Finished", atGoal())

        return output
    }

    override fun atGoal(): Boolean {
        return timer.hasElapsed(trajectory.totalTime)
    }
}