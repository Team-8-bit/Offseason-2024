package org.team9432.resources.drive.module

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.team9432.lib.util.printOnError
import org.team9432.resources.drive.DrivetrainConstants.DRIVE_CURRENT_LIMIT
import org.team9432.resources.drive.DrivetrainConstants.DRIVE_GEAR_RATIO
import org.team9432.resources.drive.DrivetrainConstants.ODOMETRY_FREQUENCY
import org.team9432.resources.drive.DrivetrainConstants.STEER_CURRENT_LIMIT
import org.team9432.resources.drive.module.ModuleIO.ModuleIOInputs
import org.team9432.resources.drive.odometrythread.OdometryThreadReal
import java.util.*


class ModuleIOKraken(private val moduleConstants: SwerveModuleConstants, canbusName: String): ModuleIO {
    private val driveMotor = TalonFX(moduleConstants.DriveMotorId, canbusName)
    private val steerMotor = TalonFX(moduleConstants.SteerMotorId, canbusName)
    private val cancoder = CANcoder(moduleConstants.CANcoderId, canbusName)

    /**** Drive Signals ****/
    private val drivePosition: StatusSignal<Double> = driveMotor.position
    private val drivePositionQueue: Queue<Double> = OdometryThreadReal.registerSignal(drivePosition)
    private val driveVelocity: StatusSignal<Double> = driveMotor.velocity
    private val driveAppliedVolts: StatusSignal<Double> = driveMotor.motorVoltage
    private val driveSupplyCurrent: StatusSignal<Double> = driveMotor.supplyCurrent
    private val driveTorqueCurrent: StatusSignal<Double> = driveMotor.torqueCurrent
    private val lowFrequencyDriveSignals = arrayOf(driveVelocity, driveAppliedVolts, driveSupplyCurrent, driveTorqueCurrent)

    /**** Steer Signals ****/
    private val steerPosition: StatusSignal<Double> = steerMotor.position
    private val steerPositionQueue: Queue<Double> = OdometryThreadReal.registerSignal(steerPosition)
    private val steerVelocity: StatusSignal<Double> = steerMotor.velocity
    private val steerAppliedVolts: StatusSignal<Double> = steerMotor.motorVoltage
    private val steerSupplyCurrent: StatusSignal<Double> = steerMotor.supplyCurrent
    private val steerTorqueCurrent: StatusSignal<Double> = steerMotor.torqueCurrent
    private val lowFrequencySteerSignals = arrayOf(steerVelocity, steerAppliedVolts, steerSupplyCurrent, steerTorqueCurrent)

    /**** CANCoder Signals ****/
    private val steerAbsolutePosition: StatusSignal<Double> = cancoder.absolutePosition
    private val lowFrequencyCANCoderSignals = arrayOf(steerAbsolutePosition)

    private val highFrequencySignals = arrayOf(drivePosition, steerPosition)

    private val voltageControl = VoltageOut(0.0).withUpdateFreqHz(0.0)
    private val driveVoltageOpenLoopControl = VoltageOut(0.0).withUpdateFreqHz(0.0)
    private val steerVoltageMotionMagicControl = MotionMagicExpoVoltage(0.0).withUpdateFreqHz(0.0)
    private val neutralControl = NeutralOut().withUpdateFreqHz(0.0)

    private val rotationsPerWheelRotation: Double = moduleConstants.DriveMotorGearRatio
    private val metersPerWheelRotation: Double = Math.PI * 2 * Units.inchesToMeters(moduleConstants.WheelRadius)
    private val driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation

    private val driveConfig = getDriveConfig(moduleConstants)
    private val steerConfig = getSteerConfig(moduleConstants)
    private val encoderConfig = getEncoderConfig(moduleConstants)

    init {
        driveMotor.configurator.apply(driveConfig).printOnError { "Drive Kraken with ID #${driveMotor.deviceID} failed config: ${it.name} ${it.description}" }
        steerMotor.configurator.apply(steerConfig).printOnError { "Steer Kraken with ID #${steerMotor.deviceID} failed config: ${it.name} ${it.description}" }
        cancoder.configurator.apply(encoderConfig).printOnError { "Swerve CANCoder with ID #${cancoder.deviceID} failed config: ${it.name} ${it.description}" }

        BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_FREQUENCY, *highFrequencySignals)

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *lowFrequencyDriveSignals, *lowFrequencySteerSignals, *lowFrequencyCANCoderSignals)

//        driveMotor.optimizeBusUtilization()
//        steerMotor.optimizeBusUtilization()
//        cancoder.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        inputs.driveConnected = BaseStatusSignal.refreshAll(*lowFrequencyDriveSignals, drivePosition).isOK
        inputs.steerConnected = BaseStatusSignal.refreshAll(*lowFrequencySteerSignals, steerPosition).isOK
        inputs.cancoderConnected = BaseStatusSignal.refreshAll(*lowFrequencyCANCoderSignals).isOK

        inputs.drivePositionRotations = drivePosition.valueAsDouble / DRIVE_GEAR_RATIO
        inputs.driveVelocityRadPerSecond = Units.rotationsToRadians(driveVelocity.valueAsDouble) / DRIVE_GEAR_RATIO
        inputs.driveAppliedVolts = driveAppliedVolts.valueAsDouble
        inputs.driveSupplyCurrentAmps = driveSupplyCurrent.valueAsDouble

        inputs.steerAbsolutePosition = Rotation2d.fromRotations(steerAbsolutePosition.valueAsDouble)
        inputs.steerPosition = Rotation2d.fromRotations(steerPosition.valueAsDouble)
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocity.valueAsDouble)
        inputs.steerAppliedVolts = steerAppliedVolts.valueAsDouble
        inputs.steerSupplyCurrentAmps = steerSupplyCurrent.valueAsDouble

        inputs.odometryDrivePositionsRotations = drivePositionQueue.map { it / DRIVE_GEAR_RATIO }.toDoubleArray()
        inputs.odometrySteerPositions = steerPositionQueue.map { Rotation2d.fromRotations(it) }.toTypedArray()

        drivePositionQueue.clear()
        steerPositionQueue.clear()
    }

    override fun runDriveVoltage(volts: Double) {
        driveMotor.setControl(voltageControl.withOutput(volts))
    }

    override fun runSteerVoltage(volts: Double) {
        steerMotor.setControl(voltageControl.withOutput(volts))
    }

    override fun runSteerPosition(angle: Rotation2d) {
        steerMotor.setControl(steerVoltageMotionMagicControl.withPosition(angle.rotations))
    }

    override fun runDriveVelocity(metersPerSecond: Double, feedforwardVolts: Double) {
        driveMotor.setControl(driveVoltageOpenLoopControl.withOutput(metersPerSecond / moduleConstants.SpeedAt12VoltsMps * 12.0))
    }

    override fun setDrivePID(p: Double, i: Double, d: Double) {
        driveConfig.Slot0.apply {
            kP = p
            kI = i
            kD = d
        }
        driveMotor.configurator.apply(driveConfig)
    }

    override fun setSteerPID(p: Double, i: Double, d: Double) {
        steerConfig.Slot0.apply {
            kP = p
            kI = i
            kD = d
        }
        steerMotor.configurator.apply(steerConfig)
    }

    override fun setDriveBrake(enable: Boolean) {
        driveMotor.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }

    override fun setSteerBrake(enable: Boolean) {
        steerMotor.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }

    override fun stop() {
        driveMotor.setControl(neutralControl)
        steerMotor.setControl(neutralControl)
    }

    private fun getDriveConfig(moduleConstants: SwerveModuleConstants) = TalonFXConfiguration().apply {
        CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT
        CurrentLimits.SupplyCurrentLimitEnable = true
        MotorOutput.Inverted = if (moduleConstants.DriveMotorInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
        MotorOutput.NeutralMode = NeutralModeValue.Brake
        TorqueCurrent.PeakForwardTorqueCurrent = 60.0
        TorqueCurrent.PeakReverseTorqueCurrent = -60.0
        CurrentLimits.StatorCurrentLimit = 60.0
        CurrentLimits.StatorCurrentLimitEnable = true
        Slot0 = moduleConstants.DriveMotorGains
    }

    private fun getSteerConfig(moduleConstants: SwerveModuleConstants) = TalonFXConfiguration().apply {
        CurrentLimits.SupplyCurrentLimit = STEER_CURRENT_LIMIT
        CurrentLimits.SupplyCurrentLimitEnable = true
        MotorOutput.Inverted = if (moduleConstants.SteerMotorInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
        MotorOutput.NeutralMode = NeutralModeValue.Brake
        CurrentLimits.StatorCurrentLimit = 30.0
        CurrentLimits.StatorCurrentLimitEnable = true
        Feedback.FeedbackRemoteSensorID = moduleConstants.CANcoderId
        Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
        Feedback.RotorToSensorRatio = moduleConstants.SteerMotorGearRatio
        MotionMagic.MotionMagicCruiseVelocity = 100.0 / moduleConstants.SteerMotorGearRatio
        MotionMagic.MotionMagicAcceleration = MotionMagic.MotionMagicCruiseVelocity / 0.1
        MotionMagic.MotionMagicExpo_kV = 0.12 * moduleConstants.SteerMotorGearRatio
        MotionMagic.MotionMagicExpo_kA = 0.1
        ClosedLoopGeneral.ContinuousWrap = true
        Slot0 = moduleConstants.SteerMotorGains
    }


    private fun getEncoderConfig(moduleConstants: SwerveModuleConstants) = CANcoderConfiguration().apply {
        MagnetSensor.MagnetOffset = moduleConstants.CANcoderOffset
    }
}