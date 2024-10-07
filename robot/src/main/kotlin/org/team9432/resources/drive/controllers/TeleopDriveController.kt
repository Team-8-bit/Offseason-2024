package org.team9432.resources.drive.controllers

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import org.team9432.Robot
import org.team9432.RobotPosition
import org.team9432.lib.dashboard.LoggedTunableNumber
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.withSign

class TeleopDriveController: GenericDriveController<ChassisSpeeds>() {
    companion object {
        private const val TABLE_KEY = "TeleopDriveController"

        private val linearDeadband by LoggedTunableNumber("$TABLE_KEY/LinearDeadband", 0.15)
        private val rotationDeadband by LoggedTunableNumber("$TABLE_KEY/RotationDeadband", 0.0)
        private val maxLinearVelocity by LoggedTunableNumber("$TABLE_KEY/MaxLinearVelocity", 5.0)
        private val maxRotationVelocityDegPerSec by LoggedTunableNumber("$TABLE_KEY/MaxRotationVelocityDegPerSec", 270.0)
    }

    private var controllerX = 0.0
    private var controllerY = 0.0
    private var controllerR = 0.0

    private val ratelimitX = SlewRateLimiter(20.0)
    private val ratelimitY = SlewRateLimiter(20.0)

    fun acceptControllerInput(x: Double, y: Double, r: Double) {
        controllerX = x
        controllerY = y
        controllerR = r
    }

    override fun calculate(): ChassisSpeeds {
        val linearSpeed = getLinearSpeed(controllerX, controllerY)
        val rotationSpeed = MathUtil.applyDeadband(controllerR, rotationDeadband)

        val invert = if (Robot.alliance == DriverStation.Alliance.Red) -1 else 1

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            ratelimitX.calculate(linearSpeed.x * maxLinearVelocity) * invert,
            ratelimitY.calculate(linearSpeed.y * maxLinearVelocity) * invert,
            rotationSpeed * Units.degreesToRadians(maxRotationVelocityDegPerSec),
            RobotPosition.currentPose.rotation
        )
    }

    private fun getLinearSpeed(xInput: Double, yInput: Double): Translation2d {
        // Apply deadband
        var linearMagnitude = MathUtil.applyDeadband(hypot(xInput, yInput), linearDeadband)
        val linearDirection = Rotation2d(xInput, yInput)

        // Square magnitude
        linearMagnitude = linearMagnitude.pow(2).withSign(linearMagnitude)

        // Calcaulate new linear velocity
        val linearVelocity =
            Pose2d(0.0, 0.0, linearDirection)
                .transformBy(Transform2d(linearMagnitude, 0.0, Rotation2d()))
                .translation

        return linearVelocity
    }

    override fun atGoal() = throw UnsupportedOperationException()
}