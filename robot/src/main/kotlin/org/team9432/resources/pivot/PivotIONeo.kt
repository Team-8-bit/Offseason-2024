package org.team9432.resources.pivot

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DutyCycleEncoder
import org.team9432.lib.unit.Angle
import org.team9432.lib.unit.inRotations
import org.team9432.lib.util.temperatureFahrenheit

class PivotIONeo(private val absoluteEncoderOffset: Angle): PivotIO {
    private val leaderMotor = CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushless) // Left Motor
    private val followerMotor = CANSparkMax(16, CANSparkLowLevel.MotorType.kBrushless) // Right Motor

    private val leaderEncoder = leaderMotor.encoder
    private val followerEncoder = followerMotor.encoder

    private val absoluteEncoder = DutyCycleEncoder(7)

    init {
        leaderMotor.enableVoltageCompensation(11.0)
        leaderMotor.setSmartCurrentLimit(30)
        leaderMotor.inverted = true
        leaderMotor.idleMode = CANSparkBase.IdleMode.kBrake
        leaderMotor.burnFlash()

        followerMotor.enableVoltageCompensation(11.0)
        followerMotor.setSmartCurrentLimit(30)
        followerMotor.inverted = true
        followerMotor.idleMode = CANSparkBase.IdleMode.kBrake
        followerMotor.burnFlash()

        followerMotor.follow(leaderMotor, true)
    }

    override fun runVoltage(volts: Double) {
        leaderMotor.setVoltage(volts)
    }

    override fun setBrakeMode(enabled: Boolean) {
        val idleMode = if (enabled) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast
        leaderMotor.setIdleMode(idleMode)
        followerMotor.setIdleMode(idleMode)
    }

    override fun updateInputs(inputs: PivotIO.PivotIOInputs) {
        inputs.leaderPositionRotations = leaderEncoder.position / reduction
        inputs.leaderVelocityRPM = leaderEncoder.velocity / reduction
        inputs.leaderAppliedVoltage = leaderMotor.appliedOutput * leaderMotor.busVoltage
        inputs.leaderSupplyCurrentAmps = leaderMotor.outputCurrent
        inputs.leaderTempFahrenheit = leaderMotor.temperatureFahrenheit

        inputs.followerPositionRotations = followerEncoder.position / reduction
        inputs.followerVelocityRPM = followerEncoder.velocity / reduction
        inputs.followerAppliedVoltage = followerMotor.appliedOutput * followerMotor.busVoltage
        inputs.followerSupplyCurrentAmps = followerMotor.outputCurrent
        inputs.followerTempFahrenheit = followerMotor.temperatureFahrenheit

        inputs.absolutePositionRotations = -(absoluteEncoder.absolutePosition - absoluteEncoderOffset.inRotations)
        inputs.absoluteEncoderConnected = absoluteEncoder.isConnected
    }
}