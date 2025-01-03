package org.team9432.resources.flywheels

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import org.team9432.lib.util.temperatureFahrenheit

class FlywheelIONeo: FlywheelIO {
    private val upperMotor = CANSparkFlex(13, CANSparkLowLevel.MotorType.kBrushless)
    private val lowerMotor = CANSparkFlex(14, CANSparkLowLevel.MotorType.kBrushless)
    private val upperEncoder = upperMotor.encoder
    private val lowerEncoder = lowerMotor.encoder

    init {
        upperMotor.inverted = false
        upperMotor.idleMode = CANSparkBase.IdleMode.kBrake
        upperMotor.enableVoltageCompensation(10.0)
        upperMotor.openLoopRampRate = 0.0
        upperMotor.setSmartCurrentLimit(40)
        upperMotor.burnFlash()

        lowerMotor.inverted = true
        lowerMotor.idleMode = CANSparkBase.IdleMode.kBrake
        lowerMotor.enableVoltageCompensation(10.0)
        lowerMotor.openLoopRampRate = 0.0
        lowerMotor.setSmartCurrentLimit(40)
        lowerMotor.burnFlash()
    }

    override fun runVoltage(upperVoltage: Double, lowerVoltage: Double) {
        upperMotor.setVoltage(upperVoltage)
        lowerMotor.setVoltage(lowerVoltage)
    }

    override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {
        inputs.upperPositionRotations = upperEncoder.position / reduction
        inputs.upperVelocityRPM = upperEncoder.velocity / reduction
        inputs.upperAppliedVoltage = upperMotor.appliedOutput * upperMotor.busVoltage
        inputs.upperSupplyCurrentAmps = upperMotor.outputCurrent
        inputs.upperTempFahrenheit = upperMotor.temperatureFahrenheit

        inputs.lowerPositionRotations = lowerEncoder.position / reduction
        inputs.lowerVelocityRPM = lowerEncoder.velocity / reduction
        inputs.lowerAppliedVoltage = lowerMotor.appliedOutput * lowerMotor.busVoltage
        inputs.lowerSupplyCurrentAmps = lowerMotor.outputCurrent
        inputs.lowerTempFahrenheit = lowerMotor.temperatureFahrenheit
    }

}