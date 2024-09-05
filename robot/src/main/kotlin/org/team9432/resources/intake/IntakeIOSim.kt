package org.team9432.resources.intake

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.team9432.Robot

class IntakeIOSim: IntakeIO {
    private val sim = DCMotorSim(DCMotor.getNEO(2), reduction, 0.002)
    private var appliedVoltage = 0.0

    override fun setVoltage(volts: Double) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0)
        sim.setInputVoltage(appliedVoltage)
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        sim.update(Robot.period)
        inputs.leaderPositionRotations = sim.angularPositionRotations
        inputs.leaderVelocityRPM = sim.angularVelocityRPM
        inputs.leaderAppliedVoltage = appliedVoltage
        inputs.leaderSupplyCurrentAmps = sim.currentDrawAmps

        inputs.followerPositionRotations = sim.angularPositionRotations
        inputs.followerVelocityRPM = sim.angularVelocityRPM
        inputs.followerAppliedVoltage = appliedVoltage
        inputs.followerSupplyCurrentAmps = sim.currentDrawAmps
    }
}