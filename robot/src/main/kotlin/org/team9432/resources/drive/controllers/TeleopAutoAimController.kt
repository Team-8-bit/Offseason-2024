package org.team9432.resources.drive.controllers

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.Logger
import org.team9432.RobotPosition
import org.team9432.lib.dashboard.LoggedTunableNumber

class TeleopAutoAimController(private val goalSupplier: () -> Rotation2d): GenericDriveController<Double>() {
    companion object {
        private const val TABLE_KEY = "TeleopAutoAimController"

        private val kP by LoggedTunableNumber("$TABLE_KEY/kP", 5.0)
        private val kD by LoggedTunableNumber("$TABLE_KEY/kD", 0.0)
        private val toleranceDegrees by LoggedTunableNumber("$TABLE_KEY/ToleranceDegrees", 2.0)
    }

    private val controller = PIDController(5.0, 0.0, 0.0).apply {
        enableContinuousInput(-Math.PI, Math.PI)
        setTolerance(Units.degreesToRadians(toleranceDegrees))
    }

    override fun calculate(): Double {
        controller.setPID(kP, 0.0, kD)

        val output = controller.calculate(
            RobotPosition.currentPose.rotation.radians,
            goalSupplier.invoke().radians
        )

        Logger.recordOutput("$TABLE_KEY/PositionErrorDegrees", controller.positionError)
        Logger.recordOutput("$TABLE_KEY/AtGoal", atGoal())

        return output
    }

    override fun atGoal(): Boolean {
        return controller.atSetpoint()
    }
}