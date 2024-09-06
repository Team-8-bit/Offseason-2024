package org.team9432.resources.swerve.module

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.team9432.resources.swerve.DriveTrainConstants.CHASSIS_MAX_VELOCITY
import org.team9432.resources.swerve.DriveTrainConstants.DRIVE_GEAR_RATIO
import org.team9432.resources.swerve.DriveTrainConstants.DRIVE_MOTOR
import org.team9432.resources.swerve.DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD
import org.team9432.resources.swerve.DriveTrainConstants.STEER_FRICTION_VOLTAGE
import org.team9432.resources.swerve.DriveTrainConstants.STEER_GEAR_RATIO
import org.team9432.resources.swerve.DriveTrainConstants.STEER_INERTIA
import org.team9432.resources.swerve.DriveTrainConstants.STEER_MOTOR
import org.team9432.resources.swerve.DriveTrainConstants.WHEEL_RADIUS_METERS
import org.team9432.resources.swerve.module.ModuleIO.ModuleIOInputs
import kotlin.math.abs

/**
 * Physics sim implementation of module IO.
 *
 *
 * Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
class ModuleIOSim: ModuleIO {
    val physicsSimulationResults = SwerveModulePhysicsSimulationResults()
    private val steerSim: DCMotorSim = DCMotorSim(STEER_MOTOR, STEER_GEAR_RATIO, STEER_INERTIA)

    private var driveAppliedVolts: Double = 0.0
    private var steerAppliedVolts: Double = 0.0

    override fun updateInputs(inputs: ModuleIOInputs) {
        inputs.hardwareConnected = true

        inputs.drivePositionRotations = physicsSimulationResults.driveWheelFinalRevolutions
        inputs.driveVelocityRadPerSecond = physicsSimulationResults.driveWheelFinalVelocityRadPerSec
        inputs.driveAppliedVolts = driveAppliedVolts
        inputs.driveCurrentAmps = abs(
            DRIVE_MOTOR.getCurrent(
                physicsSimulationResults.driveWheelFinalVelocityRadPerSec,
                driveAppliedVolts
            )
        )

        inputs.steerAbsolutePosition = Rotation2d.fromRadians(steerSim.angularPositionRad)
        inputs.steerPosition = Rotation2d.fromRadians(steerSim.angularPositionRad)
        inputs.steerVelocityRadPerSec = steerSim.angularVelocityRadPerSec
        inputs.steerAppliedVolts = steerAppliedVolts
        inputs.steerCurrentAmps = abs(steerSim.currentDrawAmps)

        inputs.odometryDrivePositionsRotations = physicsSimulationResults.odometryDriveWheelRevolutions.copyOf()
        inputs.odometrySteerPositions = physicsSimulationResults.odometrySteerPositions.copyOf()
    }

    override fun setDriveVoltage(volts: Double) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
    }

    override fun setSteerVoltage(volts: Double) {
        steerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        steerSim.setInputVoltage(if (abs(steerAppliedVolts) > STEER_FRICTION_VOLTAGE) steerAppliedVolts else 0.0)
    }

    fun updateSim(periodSecs: Double) {
        steerSim.update(periodSecs)
    }

    val simulationTorque: Double
        get() = DRIVE_MOTOR.getTorque(
            DRIVE_MOTOR.getCurrent(
                physicsSimulationResults.driveWheelFinalVelocityRadPerSec * DRIVE_GEAR_RATIO,
                driveAppliedVolts
            )
        )

    val simulationSteerFacing: Rotation2d
        get() = Rotation2d.fromRadians(steerSim.angularPositionRad)

    val simulationSwerveState: SwerveModuleState
        get() = SwerveModuleState(
            physicsSimulationResults.driveWheelFinalVelocityRadPerSec * WHEEL_RADIUS_METERS,
            simulationSteerFacing
        )

    val desiredSwerveState: SwerveModuleState
        get() = SwerveModuleState(
            driveAppliedVolts * CHASSIS_MAX_VELOCITY,
            simulationSteerFacing
        )

    /**
     * this replaces DC Motor Sim for drive wheels
     */
    class SwerveModulePhysicsSimulationResults {
        var driveWheelFinalRevolutions: Double = 0.0
        var driveWheelFinalVelocityRadPerSec: Double = 0.0

        val odometryDriveWheelRevolutions: DoubleArray = DoubleArray(SIMULATION_TICKS_IN_1_PERIOD)
        val odometrySteerPositions: Array<Rotation2d> = Array(SIMULATION_TICKS_IN_1_PERIOD) { Rotation2d() }
    }
}