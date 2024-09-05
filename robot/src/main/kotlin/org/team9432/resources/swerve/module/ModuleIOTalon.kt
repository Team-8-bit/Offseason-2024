package org.team9432.resources.swerve.module

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.controls.DutyCycleOut
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.team9432.resources.swerve.DriveTrainConstants.DRIVE_CURRENT_LIMIT
import org.team9432.resources.swerve.DriveTrainConstants.STEER_CURRENT_LIMIT
import org.team9432.resources.swerve.OdometryThread
import org.team9432.resources.swerve.module.ModuleIO.ModuleIOInputs
import java.util.*

class ModuleIOTalon(drivetrainConstants: SwerveDrivetrainConstants, moduleConstants: SwerveModuleConstants, private val name: String):
    ModuleIO {
    private val driveTalon = TalonFX(moduleConstants.DriveMotorId, drivetrainConstants.CANbusName)
    private val steerTalon = TalonFX(moduleConstants.SteerMotorId, drivetrainConstants.CANbusName)
    private val cancoder = CANcoder(moduleConstants.CANcoderId, drivetrainConstants.CANbusName)

    private val driveEncoderUngearedRevolutions: Queue<Double>
    private val driveEncoderUngearedRevolutionsPerSecond: StatusSignal<Double>
    private val driveMotorAppliedVoltage: StatusSignal<Double>
    private val driveMotorCurrent: StatusSignal<Double>

    private val steerEncoderAbsolutePositionRevolutions: Queue<Double>
    private val steerEncoderVelocityRevolutionsPerSecond: StatusSignal<Double>
    private val steerMotorAppliedVolts: StatusSignal<Double>
    private val steerMotorCurrent: StatusSignal<Double>

    private val periodicallyRefreshedSignals: Array<BaseStatusSignal>

    private val DRIVE_GEAR_RATIO: Double

    init {
        val driveConfig = moduleConstants.DriveMotorInitialConfigs
        driveConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_CURRENT_LIMIT
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true
        driveTalon.configurator.apply(driveConfig)
        driveTalon.inverted = moduleConstants.DriveMotorInverted
        setDriveBrake(true)

        val steerConfig = moduleConstants.SteerMotorInitialConfigs
        steerConfig.CurrentLimits.SupplyCurrentLimit = STEER_CURRENT_LIMIT
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true
        steerTalon.configurator.apply(steerConfig)
        steerTalon.inverted = moduleConstants.SteerMotorInverted
        setSteerBrake(true)

        val encoderConfig = moduleConstants.CANcoderInitialConfigs
        encoderConfig.MagnetSensor.MagnetOffset = moduleConstants.CANcoderOffset
        cancoder.configurator.apply(encoderConfig)

        driveEncoderUngearedRevolutions = OdometryThread.registerSignalInput(driveTalon.position)
        driveEncoderUngearedRevolutionsPerSecond = driveTalon.velocity
        driveMotorAppliedVoltage = driveTalon.motorVoltage
        driveMotorCurrent = driveTalon.supplyCurrent

        steerEncoderAbsolutePositionRevolutions = OdometryThread.registerSignalInput(cancoder.absolutePosition)
        steerEncoderVelocityRevolutionsPerSecond = cancoder.velocity
        steerMotorAppliedVolts = steerTalon.motorVoltage
        steerMotorCurrent = steerTalon.supplyCurrent

        periodicallyRefreshedSignals = arrayOf(
            driveEncoderUngearedRevolutionsPerSecond,
            driveMotorAppliedVoltage, driveMotorCurrent,
            steerEncoderVelocityRevolutionsPerSecond,
            steerMotorAppliedVolts, steerMotorCurrent
        )

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *periodicallyRefreshedSignals)
        driveTalon.optimizeBusUtilization()
        steerTalon.optimizeBusUtilization()

        this.DRIVE_GEAR_RATIO = moduleConstants.DriveMotorGearRatio
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        inputs.hardwareConnected = BaseStatusSignal.refreshAll(*periodicallyRefreshedSignals).isOK

        inputs.odometryDriveWheelRevolutions = driveEncoderUngearedRevolutions.stream()
            .mapToDouble { value: Double -> value / DRIVE_GEAR_RATIO }
            .toArray()
        driveEncoderUngearedRevolutions.clear()
        if (inputs.odometryDriveWheelRevolutions.isNotEmpty()) inputs.driveWheelFinalRevolutions = inputs.odometryDriveWheelRevolutions[inputs.odometryDriveWheelRevolutions.size - 1]

        inputs.odometrySteerPositions = steerEncoderAbsolutePositionRevolutions.map { canCoderReadingRotations: Double -> this.getSteerFacingFromCANCoderReading(canCoderReadingRotations) }.toTypedArray()
        steerEncoderAbsolutePositionRevolutions.clear()
        if (inputs.odometrySteerPositions.isNotEmpty()) inputs.steerFacing = inputs.odometrySteerPositions[inputs.odometrySteerPositions.size - 1]

        inputs.driveWheelFinalVelocityRevolutionsPerSec = driveEncoderUngearedRevolutionsPerSecond.valueAsDouble / DRIVE_GEAR_RATIO
        inputs.driveMotorAppliedVolts = driveMotorAppliedVoltage.valueAsDouble
        inputs.driveMotorCurrentAmps = driveMotorCurrent.valueAsDouble

        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerEncoderVelocityRevolutionsPerSecond.valueAsDouble)
        inputs.steerMotorAppliedVolts = steerMotorAppliedVolts.valueAsDouble
        inputs.steerMotorCurrentAmps = steerMotorCurrent.valueAsDouble
    }

    private fun getSteerFacingFromCANCoderReading(canCoderReadingRotations: Double): Rotation2d {
        return Rotation2d.fromRotations(canCoderReadingRotations)
    }

    override fun setDriveVoltage(volts: Double) {
        val voltageOut = VoltageOut(volts).withEnableFOC(false)
        driveTalon.setControl(voltageOut)
    }

    override fun setSteerPowerPercent(powerPercent: Double) {
        steerTalon.setControl(
            DutyCycleOut(powerPercent)
                .withEnableFOC(true)
        )
    }

    override fun setDriveBrake(enable: Boolean) {
        driveTalon.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }

    override fun setSteerBrake(enable: Boolean) {
        steerTalon.setNeutralMode(if (enable) NeutralModeValue.Brake else NeutralModeValue.Coast)
    }
}