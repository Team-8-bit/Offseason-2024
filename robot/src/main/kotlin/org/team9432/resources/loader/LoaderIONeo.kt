package org.team9432.resources.loader

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import org.team9432.lib.util.temperatureFahrenheit

class LoaderIONeo: LoaderIO {
    private val motor = CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless)
    private val encoder = motor.encoder

    init {
        motor.inverted = false
        motor.enableVoltageCompensation(11.0)
        motor.setSmartCurrentLimit(20)
        motor.idleMode = CANSparkBase.IdleMode.kBrake
    }

    override fun setVoltage(volts: Double) {
        motor.setVoltage(volts)
    }

    override fun updateInputs(inputs: LoaderIO.LoaderIOInputs) {
        inputs.positionRotations = encoder.position / reduction
        inputs.velocityRPM = encoder.velocity / reduction
        inputs.appliedVoltage = motor.appliedOutput * motor.busVoltage
        inputs.supplyCurrentAmps = motor.outputCurrent
        inputs.tempFahrenheit = motor.temperatureFahrenheit
    }
}