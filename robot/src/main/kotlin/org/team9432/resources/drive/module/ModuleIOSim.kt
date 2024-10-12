package org.team9432.resources.drive.module

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.team9432.lib.simulation.SimulatedSwerveModule
import org.team9432.resources.drive.DrivetrainConstants.CHASSIS_MAX_VELOCITY
import org.team9432.resources.drive.DrivetrainConstants.DRIVE_GEAR_RATIO
import org.team9432.resources.drive.DrivetrainConstants.DRIVE_MOTOR
import org.team9432.resources.drive.DrivetrainConstants.STEER_FRICTION_VOLTAGE
import org.team9432.resources.drive.DrivetrainConstants.STEER_GEAR_RATIO
import org.team9432.resources.drive.DrivetrainConstants.STEER_INERTIA
import org.team9432.resources.drive.DrivetrainConstants.STEER_MOTOR
import org.team9432.resources.drive.DrivetrainConstants.WHEEL_RADIUS_METERS
import org.team9432.resources.drive.module.ModuleIO.ModuleIOInputs
import kotlin.math.abs

class ModuleIOSim: ModuleIO, SimulatedSwerveModule() {
    private val steerSim: DCMotorSim = DCMotorSim(STEER_MOTOR, STEER_GEAR_RATIO, STEER_INERTIA)

    private val driveFeedback: PIDController = PIDController(1.0, 0.0, 0.0).apply {
        setTolerance(0.25)
    }
    private val steerFeedback: PIDController = PIDController(8.0, 0.0, 0.2)

    private var driveAppliedVolts: Double = 0.0
    private var steerAppliedVolts: Double = 0.0

    private var angleSetpoint: Rotation2d? = null
    private var speedSetpoint: Double? = null
    private var speedFeedforward: Double = 0.0

    init {
        steerFeedback.enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        speedSetpoint?.let { targetSpeedMPS -> setDriveMotor(driveFeedback.calculate(inputs.driveVelocityRadPerSecond, targetSpeedMPS / WHEEL_RADIUS_METERS) + speedFeedforward) }
        angleSetpoint?.let { targetAngle -> setSteerMotor(steerFeedback.calculate(inputs.steerAbsolutePosition.radians, targetAngle.radians)) }

        inputs.driveConnected = true
        inputs.steerConnected = true
        inputs.cancoderConnected = true

        inputs.drivePositionRotations = physicsSimulationResults.driveWheelFinalRevolutions
        inputs.driveVelocityRadPerSecond = physicsSimulationResults.driveWheelFinalVelocityRadPerSec
        inputs.driveAppliedVolts = driveAppliedVolts
        inputs.driveSupplyCurrentAmps = abs(
            DRIVE_MOTOR.getCurrent(
                physicsSimulationResults.driveWheelFinalVelocityRadPerSec,
                driveAppliedVolts
            )
        )

        inputs.steerAbsolutePosition = Rotation2d.fromRadians(steerSim.angularPositionRad)
        inputs.steerPosition = Rotation2d.fromRadians(steerSim.angularPositionRad)
        inputs.steerVelocityRadPerSec = steerSim.angularVelocityRadPerSec
        inputs.steerAppliedVolts = steerAppliedVolts
        inputs.steerSupplyCurrentAmps = abs(steerSim.currentDrawAmps)

        inputs.odometryDrivePositionsRotations = physicsSimulationResults.odometryDriveWheelRevolutions.copyOf()
        inputs.odometrySteerPositions = physicsSimulationResults.odometrySteerPositions.copyOf()
    }

    override fun runDriveVoltage(volts: Double) {
        speedSetpoint = null
        setDriveMotor(volts)
    }

    override fun runSteerVoltage(volts: Double) {
        angleSetpoint = null
        setSteerMotor(volts)
    }

    private fun setDriveMotor(volts: Double) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
    }

    private fun setSteerMotor(volts: Double) {
        steerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0)
        steerSim.setInputVoltage(if (abs(steerAppliedVolts) > STEER_FRICTION_VOLTAGE) steerAppliedVolts else 0.0)
    }

    override fun runSteerPosition(angle: Rotation2d) {
        angleSetpoint = angle
    }

//    override fun runDriveVelocity(metersPerSecond: Double, feedforwardVolts: Double) {
//        speedFeedforward = feedforwardVolts
//        speedSetpoint = metersPerSecond
//    }

    override fun setDrivePID(p: Double, i: Double, d: Double) {
        driveFeedback.apply {
            this.p = p
            this.i = i
            this.d = d
        }
    }

    override fun setSteerPID(p: Double, i: Double, d: Double) {
        steerFeedback.apply {
            this.p = p
            this.i = i
            this.d = d
        }
    }

    /**** Simulation Methods ****/

    override fun updateSteerSim(periodSecs: Double) {
        steerSim.update(periodSecs)
    }

    override fun getSimulationTorque(): Double {
        val currentAmps = DRIVE_MOTOR.getCurrent(physicsSimulationResults.driveWheelFinalVelocityRadPerSec * DRIVE_GEAR_RATIO, driveAppliedVolts)
        return DRIVE_MOTOR.getTorque(currentAmps)
    }

    val simulationSteerFacing: Rotation2d
        get() = Rotation2d.fromRadians(steerSim.angularPositionRad)

    override fun getSimulationSwerveState(): SwerveModuleState {
        return SwerveModuleState(
            physicsSimulationResults.driveWheelFinalVelocityRadPerSec * WHEEL_RADIUS_METERS,
            simulationSteerFacing
        )
    }

    override fun getDesiredSimulationSwerveState(): SwerveModuleState {
        return SwerveModuleState(driveAppliedVolts * CHASSIS_MAX_VELOCITY, simulationSteerFacing)
    }

}