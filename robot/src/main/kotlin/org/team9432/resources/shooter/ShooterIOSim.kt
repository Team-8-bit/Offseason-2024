package org.team9432.resources.shooter

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.team9432.Robot

class ShooterIOSim: ShooterIO {
    private val upperSim = DCMotorSim(DCMotor.getNeoVortex(1), reduction, 0.001)
    private val lowerSim = DCMotorSim(DCMotor.getNeoVortex(1), reduction, 0.001)

    private val ff = SimpleMotorFeedforward(0.0, 0.0021, 0.0)

    private var upperAppliedVoltage = 0.0
    private var lowerAppliedVoltage = 0.0

    override fun runVoltage(upperVoltage: Double, lowerVoltage: Double) {
        upperAppliedVoltage = MathUtil.clamp(upperVoltage, -12.0, 12.0)
        lowerAppliedVoltage = MathUtil.clamp(lowerVoltage, -12.0, 12.0)

        upperSim.setInputVoltage(upperAppliedVoltage)
        lowerSim.setInputVoltage(lowerAppliedVoltage)
    }

    override fun runVelocity(upperRPM: Double, lowerRPM: Double) {
        upperAppliedVoltage = MathUtil.clamp(ff.calculate(upperRPM), -12.0, 12.0)
        lowerAppliedVoltage = MathUtil.clamp(ff.calculate(lowerRPM), -12.0, 12.0)

        upperSim.setInputVoltage(upperAppliedVoltage)
        lowerSim.setInputVoltage(upperAppliedVoltage)
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
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