package org.team9432.resources.loader

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax

object LoaderIONeo: LoaderIO {
    private val motor = CANSparkMax(12, CANSparkLowLevel.MotorType.kBrushless)

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
        inputs.motorSpeed = 2.0
    }
}