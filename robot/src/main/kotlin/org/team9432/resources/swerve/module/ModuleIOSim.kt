package org.team9432.resources.swerve.module

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.PIDController
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

class ModuleIOSim: ModuleIO {
    val physicsSimulationResults = SwerveModulePhysicsSimulationResults()
    private val steerSim: DCMotorSim = DCMotorSim(STEER_MOTOR, STEER_GEAR_RATIO, STEER_INERTIA)

    private val driveFeedback: PIDController = PIDController(2.1, 0.0, 0.0)
    private val steerFeedback: PIDController = PIDController(10.0, 0.0, 0.0)

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

    override fun runDriveVelocity(metersPerSecond: Double, feedforwardVolts: Double) {
        speedFeedforward = feedforwardVolts
        speedSetpoint = metersPerSecond
    }

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