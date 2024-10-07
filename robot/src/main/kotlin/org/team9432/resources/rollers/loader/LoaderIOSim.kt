package org.team9432.resources.rollers.loader

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import org.team9432.Robot

class LoaderIOSim: LoaderIO {
    private val sim = DCMotorSim(DCMotor.getNEO(1), reduction, 0.001)
    private var appliedVoltage = 0.0

    override fun setVoltage(volts: Double) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0)
        sim.setInputVoltage(appliedVoltage)
    }

    override fun updateInputs(inputs: LoaderIO.LoaderIOInputs) {
        sim.update(Robot.periodSeconds)
        inputs.positionRotations = sim.angularPositionRotations
        inputs.velocityRPM = sim.angularVelocityRPM
        inputs.appliedVoltage = appliedVoltage
        inputs.supplyCurrentAmps = sim.currentDrawAmps
    }
}