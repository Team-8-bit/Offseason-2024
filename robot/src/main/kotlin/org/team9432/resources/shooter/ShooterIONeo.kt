package org.team9432.resources.shooter

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import org.team9432.lib.util.temperatureFahrenheit

class ShooterIONeo: ShooterIO {
    private val upperMotor = CANSparkFlex(14, CANSparkLowLevel.MotorType.kBrushless)
    private val lowerMotor = CANSparkFlex(13, CANSparkLowLevel.MotorType.kBrushless)
    private val upperEncoder = upperMotor.encoder
    private val lowerEncoder = lowerMotor.encoder

    private val ff = SimpleMotorFeedforward(0.0, 0.0021, 0.0)

    init {
        upperMotor.inverted = false
        upperMotor.idleMode = CANSparkBase.IdleMode.kBrake
        upperMotor.enableVoltageCompensation(10.0)
        upperMotor.openLoopRampRate = 0.0
        upperMotor.setSmartCurrentLimit(25)

        lowerMotor.inverted = true
        lowerMotor.idleMode = CANSparkBase.IdleMode.kBrake
        lowerMotor.enableVoltageCompensation(10.0)
        lowerMotor.openLoopRampRate = 0.0
        lowerMotor.setSmartCurrentLimit(25)
    }

    override fun runVoltage(upperVoltage: Double, lowerVoltage: Double) {
        upperMotor.setVoltage(upperVoltage)
        lowerMotor.setVoltage(lowerVoltage)
    }

    override fun runVelocity(upperRPM: Double, lowerRPM: Double) {
        upperMotor.setVoltage(ff.calculate(upperRPM))
        lowerMotor.setVoltage(ff.calculate(lowerRPM))
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
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