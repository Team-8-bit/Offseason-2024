package org.team9432.resources.pivot

import org.team9432.annotation.Logged

interface PivotIO {
    @Logged
    open class PivotIOInputs {
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

    val reduction get() = (5.0 / 1.0) * (50.0 / 18.0) * (60.0 / 20.0)

    fun updateInputs(inputs: PivotIOInputs) {}

    fun runVoltage(volts: Double) {}

    fun setBrakeMode(enabled: Boolean) {}
}