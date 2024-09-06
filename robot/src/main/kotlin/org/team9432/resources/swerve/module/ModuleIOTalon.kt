package org.team9432.resources.swerve.module

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.team9432.resources.swerve.DriveTrainConstants.DRIVE_CURRENT_LIMIT
import org.team9432.resources.swerve.DriveTrainConstants.DRIVE_GEAR_RATIO
import org.team9432.resources.swerve.DriveTrainConstants.ODOMETRY_FREQUENCY
import org.team9432.resources.swerve.DriveTrainConstants.STEER_CURRENT_LIMIT
import org.team9432.resources.swerve.DriveTrainConstants.STEER_GEAR_RATIO
import org.team9432.resources.swerve.OdometryThreadReal
import org.team9432.resources.swerve.module.ModuleIO.ModuleIOInputs
import java.util.*


class ModuleIOTalon(drivetrainConstants: SwerveDrivetrainConstants, moduleConstants: SwerveModuleConstants): ModuleIO {
    private val driveTalon = TalonFX(moduleConstants.DriveMotorId, drivetrainConstants.CANbusName)
    private val steerTalon = TalonFX(moduleConstants.SteerMotorId, drivetrainConstants.CANbusName)
    private val cancoder = CANcoder(moduleConstants.CANcoderId, drivetrainConstants.CANbusName)

    private val drivePosition: StatusSignal<Double>
    private val drivePositionQueue: Queue<Double>
    private val driveVelocity: StatusSignal<Double>
    private val driveAppliedVolts: StatusSignal<Double>
    private val driveCurrent: StatusSignal<Double>

    private val steerAbsolutePosition: StatusSignal<Double>
    private val steerPosition: StatusSignal<Double>
    private val steerPositionQueue: Queue<Double>
    private val steerVelocity: StatusSignal<Double>
    private val steerAppliedVolts: StatusSignal<Double>
    private val steerCurrent: StatusSignal<Double>

    init {
        val driveConfig = TalonFXConfiguration()
        driveConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true
        driveConfig.MotorOutput.Inverted = if (moduleConstants.DriveMotorInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
        driveTalon.configurator.apply(driveConfig)
        setDriveBrake(true)

        val steerConfig = TalonFXConfiguration()
        steerConfig.CurrentLimits.SupplyCurrentLimit = STEER_CURRENT_LIMIT
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true
        steerConfig.MotorOutput.Inverted = if (moduleConstants.SteerMotorInverted) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive
        steerTalon.configurator.apply(steerConfig)
        setSteerBrake(true)

        val encoderConfig = CANcoderConfiguration()
        encoderConfig.MagnetSensor.MagnetOffset = moduleConstants.CANcoderOffset
        cancoder.configurator.apply(encoderConfig)

        drivePosition = driveTalon.position
        drivePositionQueue = OdometryThreadReal.registerSignal(drivePosition)
        driveVelocity = driveTalon.velocity
        driveAppliedVolts = driveTalon.motorVoltage
        driveCurrent = driveTalon.supplyCurrent

        steerAbsolutePosition = cancoder.absolutePosition
        steerPosition = steerTalon.position
        steerPositionQueue = OdometryThreadReal.registerSignal(steerPosition)
        steerVelocity = steerTalon.velocity
        steerAppliedVolts = steerTalon.motorVoltage
        steerCurrent = steerTalon.supplyCurrent

        BaseStatusSignal.setUpdateFrequencyForAll(ODOMETRY_FREQUENCY, drivePosition, steerPosition)

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent, steerAbsolutePosition, steerVelocity, steerAppliedVolts, steerCurrent)

        driveTalon.optimizeBusUtilization()
        steerTalon.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        inputs.hardwareConnected = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent, steerAbsolutePosition, steerPosition, steerVelocity, steerAppliedVolts, steerCurrent).isOK

        inputs.drivePositionRotations = drivePosition.valueAsDouble / DRIVE_GEAR_RATIO
        inputs.driveVelocityRadPerSecond = Units.rotationsToRadians(driveVelocity.valueAsDouble) / DRIVE_GEAR_RATIO
        inputs.driveAppliedVolts = driveAppliedVolts.valueAsDouble
        inputs.driveCurrentAmps = driveCurrent.valueAsDouble

        inputs.steerAbsolutePosition = Rotation2d.fromRotations(steerAbsolutePosition.valueAsDouble)
        inputs.steerPosition = Rotation2d.fromRotations(steerPosition.valueAsDouble / STEER_GEAR_RATIO)
        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerVelocity.valueAsDouble) / STEER_GEAR_RATIO
        inputs.steerAppliedVolts = steerAppliedVolts.valueAsDouble
        inputs.steerCurrentAmps = steerCurrent.valueAsDouble

        inputs.odometryDrivePositionsRotations = drivePositionQueue.map { Units.rotationsToRadians(it) / DRIVE_GEAR_RATIO }.toDoubleArray()
        inputs.odometrySteerPositions = steerPositionQueue.map { Rotation2d.fromRotations(it / STEER_GEAR_RATIO) }.toTypedArray()

        drivePositionQueue.clear()
        steerPositionQueue.clear()
    }

    override fun setDriveVoltage(volts: Double) {
        driveTalon.setControl(VoltageOut(volts).withEnableFOC(false))
    }

    override fun setSteerVoltage(volts: Double) {
        steerTalon.setControl(VoltageOut(volts).withEnableFOC(true))
    }

    override fun setDriveBrake(enable: Boolean) {
        driveTalon.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }

    override fun setSteerBrake(enable: Boolean) {
        steerTalon.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }
}