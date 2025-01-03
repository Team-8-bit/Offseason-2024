package org.team9432.resources.rollers.intake

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import org.team9432.lib.util.temperatureFahrenheit

class IntakeIONeo: IntakeIO {
    // Upper Motor
    private val leaderMotor = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    // Lower Motor
    private val followerMotor = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)

    private val leaderEncoder = leaderMotor.encoder
    private val followerEncoder = followerMotor.encoder

    init {
        leaderMotor.inverted = true
        leaderMotor.enableVoltageCompensation(11.0)
        leaderMotor.setSmartCurrentLimit(30)
        leaderMotor.idleMode = CANSparkBase.IdleMode.kBrake
        leaderMotor.openLoopRampRate = 0.25
        leaderMotor.burnFlash()

        followerMotor.inverted = true
        followerMotor.enableVoltageCompensation(11.0)
        followerMotor.setSmartCurrentLimit(30)
        followerMotor.idleMode = CANSparkBase.IdleMode.kBrake
        followerMotor.openLoopRampRate = 0.25
        followerMotor.burnFlash()

        followerMotor.follow(leaderMotor)
    }

    override fun setVoltage(volts: Double) {
        leaderMotor.setVoltage(volts)
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
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
    }
}