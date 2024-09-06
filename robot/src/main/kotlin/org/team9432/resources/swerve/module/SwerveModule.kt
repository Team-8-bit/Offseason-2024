package org.team9432.resources.swerve.module

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.Logger
import org.team9432.Robot
import org.team9432.lib.coroutines.Team8BitRobot.Runtime.*
import org.team9432.resources.swerve.DriveTrainConstants
import kotlin.math.cos


class SwerveModule(private val io: ModuleIO, private val name: String) {
    private val inputs: LoggedModuleIOInputs = LoggedModuleIOInputs()

    private val driveFeedforward: SimpleMotorFeedforward
    private val driveFeedback: PIDController
    private val turnFeedback: PIDController

    private var angleSetpoint: Rotation2d? = null
    private var speedSetpoint: Double? = null
    private var turnRelativeOffset: Rotation2d? = null

    var odometryPositions: Array<SwerveModulePosition> = emptyArray()
        private set

    init {
        when (Robot.runtime) {
            REAL, REPLAY -> {
                driveFeedforward = SimpleMotorFeedforward(0.1, 0.13)
                driveFeedback = PIDController(0.05, 0.0, 0.0)
                turnFeedback = PIDController(7.0, 0.0, 0.0)
            }

            SIM -> {
                driveFeedforward = SimpleMotorFeedforward(0.1, 0.13)
                driveFeedback = PIDController(0.1, 0.0, 0.0)
                turnFeedback = PIDController(10.0, 0.0, 0.0)
            }
        }

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI)

        io.setDriveBrake(true)
        io.setSteerBrake(true)
    }

    fun updateInputs() = io.updateInputs(inputs)

    fun periodic() {
        Logger.processInputs("Drive/Module-$name", inputs)

        // On first cycle, reset relative turn encoder
        // Wait until absolute angle is nonzero in case it wasn't initialized yet
        if (turnRelativeOffset == null && inputs.steerAbsolutePosition.radians != 0.0) {
            turnRelativeOffset = inputs.steerAbsolutePosition.minus(inputs.steerPosition);
        }

        trackClosedLoopStates()

        odometryPositions = Array(inputs.odometryDrivePositionsRotations.size) { i ->
            val positionMeters = driveWheelRotationsToMeters(inputs.odometryDrivePositionsRotations[i])
            val angle: Rotation2d = inputs.odometrySteerPositions[i]
            SwerveModulePosition(positionMeters, angle)
        }
    }

    private fun trackClosedLoopStates() {
        val angleTarget = angleSetpoint
        val speedTarget = speedSetpoint
        // Run closed loop turn control
        if (angleTarget != null) {
            io.setSteerVoltage(turnFeedback.calculate(angle.radians, angleTarget.radians))

            // Run closed loop drive control
            // Only if closed loop turn control is running
            if (speedTarget != null) {
                // Scale velocity based on turn error

                // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                val adjustSpeedSetpoint = speedSetpoint!! * cos(turnFeedback.positionError)

                // Run drive controller
                val velocityRadPerSec: Double = adjustSpeedSetpoint / DriveTrainConstants.WHEEL_RADIUS_METERS
                io.setDriveVoltage(
                    driveFeedforward.calculate(velocityRadPerSec) +
                            driveFeedback.calculate(inputs.driveVelocityRadPerSecond, velocityRadPerSec)
                )
            }
        }
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized state.
     */
    fun runSetpoint(state: SwerveModuleState): SwerveModuleState {
        val optimizedSetpoint = SwerveModuleState.optimize(state, angle)

        this.angleSetpoint = optimizedSetpoint.angle
        this.speedSetpoint = optimizedSetpoint.speedMetersPerSecond

        return optimizedSetpoint
    }

    /** The current turn angle of the module. */
    val angle: Rotation2d
        get() = inputs.steerAbsolutePosition

    val steerVelocityRadPerSec: Double
        get() = inputs.steerVelocityRadPerSec

    val drivePositionMeters: Double
        /**
         * Returns the current drive position of the module in meters.
         */
        get() = driveWheelRotationsToMeters(inputs.drivePositionRotations)

    private fun driveWheelRotationsToMeters(driveWheelRotations: Double): Double {
        return Units.rotationsToRadians(driveWheelRotations) * DriveTrainConstants.WHEEL_RADIUS_METERS
    }

    val driveVelocityMetersPerSec: Double
        /**
         * Returns the current drive velocity of the module in meters per second.
         */
        get() = driveWheelRotationsToMeters(inputs.driveVelocityRadPerSecond)

    val latestPosition: SwerveModulePosition
        /**
         * Returns the module position (turn angle and drive position).
         */
        get() = SwerveModulePosition(drivePositionMeters, angle)

    val measuredState: SwerveModuleState
        /**
         * Returns the module state (turn angle and drive velocity).
         */
        get() = SwerveModuleState(driveVelocityMetersPerSec, angle)
}