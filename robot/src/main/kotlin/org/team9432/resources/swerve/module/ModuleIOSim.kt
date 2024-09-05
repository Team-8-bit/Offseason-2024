package org.team9432.resources.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
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
    val physicsSimulationResults: SwerveModulePhysicsSimulationResults
    private val steerSim: DCMotorSim
    var appliedVolts: Double = 0.0
        private set
    private var steerAppliedVolts: Double = 0.0

    init {
        this.steerSim = DCMotorSim(STEER_MOTOR, STEER_GEAR_RATIO, STEER_INERTIA)

        this.physicsSimulationResults = SwerveModulePhysicsSimulationResults()
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        inputs.driveWheelFinalRevolutions = physicsSimulationResults.driveWheelFinalRevolutions
        inputs.driveWheelFinalVelocityRevolutionsPerSec = Units.radiansToRotations(physicsSimulationResults.driveWheelFinalVelocityRadPerSec)
        inputs.driveMotorAppliedVolts = appliedVolts
        inputs.driveMotorCurrentAmps = abs(
            DRIVE_MOTOR.getCurrent(
                physicsSimulationResults.driveWheelFinalVelocityRadPerSec,
                appliedVolts
            )
        )

        inputs.steerFacing = Rotation2d.fromRadians(steerSim.angularPositionRad)
        inputs.steerVelocityRadPerSec = steerSim.angularVelocityRadPerSec
        inputs.steerMotorAppliedVolts = steerAppliedVolts
        inputs.steerMotorCurrentAmps = abs(steerSim.currentDrawAmps)

        inputs.odometryDriveWheelRevolutions = physicsSimulationResults.odometryDriveWheelRevolutions.copyOf()
        inputs.odometrySteerPositions = physicsSimulationResults.odometrySteerPositions.copyOf()

        inputs.hardwareConnected = true
    }


    override fun setDriveVoltage(volts: Double) {
        appliedVolts = volts
    }

    override fun setSteerPowerPercent(powerPercent: Double) {
        steerAppliedVolts = (powerPercent * 12)
        steerSim.setInputVoltage(if (abs(steerAppliedVolts) > STEER_FRICTION_VOLTAGE) steerAppliedVolts else 0.0)
    }

    fun updateSim(periodSecs: Double) {
        steerSim.update(periodSecs)
    }

    val simulationTorque: Double
        get() = DRIVE_MOTOR.getTorque(
            DRIVE_MOTOR.getCurrent(
                physicsSimulationResults.driveWheelFinalVelocityRadPerSec * DRIVE_GEAR_RATIO,
                appliedVolts
            )
        )

    val simulationSteerFacing: Rotation2d
        get() {
            return Rotation2d.fromRadians(steerSim.angularPositionRad)
        }

    val simulationSwerveState: SwerveModuleState
        get() {
            return SwerveModuleState(
                physicsSimulationResults.driveWheelFinalVelocityRadPerSec * WHEEL_RADIUS_METERS,
                simulationSteerFacing
            )
        }

    val desiredSwerveState: SwerveModuleState
        get() {
            return SwerveModuleState(
                appliedVolts * CHASSIS_MAX_VELOCITY,
                simulationSteerFacing
            )
        }

    /**
     * this replaces DC Motor Sim for drive wheels
     */
    class SwerveModulePhysicsSimulationResults {
        @JvmField
        var driveWheelFinalRevolutions: Double = 0.0
        var driveWheelFinalVelocityRadPerSec: Double = 0.0

        val odometryDriveWheelRevolutions: DoubleArray = DoubleArray(SIMULATION_TICKS_IN_1_PERIOD)
        val odometrySteerPositions: Array<Rotation2d> = Array(SIMULATION_TICKS_IN_1_PERIOD) { Rotation2d() }
    }
}