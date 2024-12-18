package org.team9432.resources.flywheels

import org.team9432.annotation.Logged

interface FlywheelIO {
    @Logged
    open class FlywheelIOInputs {
        var upperPositionRotations: Double = 0.0
        var upperVelocityRPM: Double = 0.0
        var upperAppliedVoltage: Double = 0.0
        var upperSupplyCurrentAmps: Double = 0.0
        var upperTempFahrenheit: Double = 0.0
        var lowerPositionRotations: Double = 0.0
        var lowerVelocityRPM: Double = 0.0
        var lowerAppliedVoltage: Double = 0.0
        var lowerSupplyCurrentAmps: Double = 0.0
        var lowerTempFahrenheit: Double = 0.0
    }

    val reduction get() = 1.0

    fun runVoltage(upperVoltage: Double, lowerVoltage: Double) {}

    fun updateInputs(inputs: FlywheelIOInputs) {}
}