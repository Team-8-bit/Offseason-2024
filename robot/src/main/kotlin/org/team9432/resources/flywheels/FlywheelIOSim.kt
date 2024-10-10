package org.team9432.resources.flywheels

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.team9432.Robot

class FlywheelIOSim: FlywheelIO {
    private val upperSim = DCMotorSim(DCMotor.getNeoVortex(1), reduction, 0.001)
    private val lowerSim = DCMotorSim(DCMotor.getNeoVortex(1), reduction, 0.001)

    private var upperAppliedVoltage = 0.0
    private var lowerAppliedVoltage = 0.0

    override fun runVoltage(upperVoltage: Double, lowerVoltage: Double) {
        upperAppliedVoltage = MathUtil.clamp(upperVoltage, -12.0, 12.0)
        lowerAppliedVoltage = MathUtil.clamp(lowerVoltage, -12.0, 12.0)

        upperSim.setInputVoltage(upperAppliedVoltage)
        lowerSim.setInputVoltage(lowerAppliedVoltage)
    }

    override fun updateInputs(inputs: FlywheelIO.FlywheelIOInputs) {
        upperSim.update(Robot.periodSeconds)
        lowerSim.update(Robot.periodSeconds)

        inputs.upperPositionRotations = upperSim.angularPositionRotations
        inputs.upperVelocityRPM = upperSim.angularVelocityRPM
        inputs.upperAppliedVoltage = upperAppliedVoltage
        inputs.upperSupplyCurrentAmps = upperSim.currentDrawAmps

        inputs.lowerPositionRotations = lowerSim.angularPositionRotations
        inputs.lowerVelocityRPM = lowerSim.angularVelocityRPM
        inputs.lowerAppliedVoltage = lowerAppliedVoltage
        inputs.lowerSupplyCurrentAmps = lowerSim.currentDrawAmps
    }

}