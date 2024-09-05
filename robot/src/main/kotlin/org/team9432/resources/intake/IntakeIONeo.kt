package org.team9432.resources.intake

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import org.team9432.lib.util.temperatureFahrenheit

object IntakeIONeo: IntakeIO {
    private val leader = CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushless)
    private val follower = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)
    private val leaderEncoder = leader.encoder
    private val followerEncoder = follower.encoder
    private val reduction = 30.0 / 12.0

    init {
        leader.inverted = true
        leader.enableVoltageCompensation(11.0)
        leader.setSmartCurrentLimit(20)
        leader.idleMode = CANSparkBase.IdleMode.kBrake

        follower.inverted = true
        follower.enableVoltageCompensation(11.0)
        follower.setSmartCurrentLimit(20)
        follower.idleMode = CANSparkBase.IdleMode.kBrake

        follower.follow(leader)
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.leaderPositionRotations = leaderEncoder.position / reduction
        inputs.leaderVelocityRPM = leaderEncoder.velocity / reduction
        inputs.leaderAppliedVoltage = leader.appliedOutput * leader.busVoltage
        inputs.leaderSupplyCurrentAmps = leader.outputCurrent
        inputs.leaderTempFahrenheit = leader.temperatureFahrenheit

        inputs.followerPositionRotations = followerEncoder.position / reduction
        inputs.followerVelocityRPM = followerEncoder.velocity / reduction
        inputs.followerAppliedVoltage = follower.appliedOutput * follower.busVoltage
        inputs.followerSupplyCurrentAmps = follower.outputCurrent
        inputs.followerTempFahrenheit = follower.temperatureFahrenheit
    }

    override fun setVoltage(volts: Double) {
        leader.setVoltage(volts)
    }
}