package org.team9432.resources.pivot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import org.team9432.Robot

class PivotIOSim: PivotIO {
    private val sim = SingleJointedArmSim(
        DCMotor.getNEO(2),
        reduction,
        0.75,
        20.0,
        Units.degreesToRadians(0.0),
        Units.degreesToRadians(50.69),
        true,
        Units.degreesToRadians(0.0)
    )
    private var appliedVoltage = 0.0

    override fun runVoltage(volts: Double) {
        appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0)
        sim.setInputVoltage(appliedVoltage)
    }

    override fun updateInputs(inputs: PivotIO.PivotIOInputs) {
        sim.update(Robot.periodSeconds)
        inputs.leaderPositionRotations = Units.radiansToRotations(sim.angleRads)
        inputs.leaderVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(sim.velocityRadPerSec)
        inputs.leaderAppliedVoltage = appliedVoltage
        inputs.leaderSupplyCurrentAmps = sim.currentDrawAmps

        inputs.followerPositionRotations = Units.radiansToRotations(sim.angleRads)
        inputs.followerVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(sim.velocityRadPerSec)
        inputs.followerAppliedVoltage = appliedVoltage
        inputs.followerSupplyCurrentAmps = sim.currentDrawAmps
    }
}