package org.team9432.resources.intake

import org.team9432.annotation.Logged

interface IntakeIO {
    @Logged
    open class IntakeIOInputs {
        var leaderPositionRotations: Double = 0.0
        var leaderVelocityRPM: Double = 0.0
        var leaderAppliedVoltage: Double = 0.0
        var leaderSupplyCurrentAmps: Double = 0.0
        var leaderTempFahrenheit: Double = 0.0
        var followerPositionRotations: Double = 0.0
        var followerVelocityRPM: Double = 0.0
        var followerAppliedVoltage: Double = 0.0
        var followerSupplyCurrentAmps: Double = 0.0
        var followerTempFahrenheit: Double = 0.0
    }

    val reduction get() = 30.0 / 12.0

    fun setVoltage(volts: Double)

    fun updateInputs(inputs: IntakeIOInputs)
}