package org.team9432.resources.drive.odometrythread

import edu.wpi.first.wpilibj.Timer
import org.team9432.Robot
import org.team9432.lib.simulation.SIMULATION_TICKS_IN_1_PERIOD

class OdometryThreadSim: OdometryThread {
    override fun updateInputs(inputs: OdometryThread.OdometryThreadInputs) {
        inputs.measurementTimestamps = DoubleArray(SIMULATION_TICKS_IN_1_PERIOD)
        val robotStartingTimeStamps: Double = Timer.getFPGATimestamp()
        val iterationPeriodSeconds: Double = Robot.period / SIMULATION_TICKS_IN_1_PERIOD
        for (i in 0 until SIMULATION_TICKS_IN_1_PERIOD) inputs.measurementTimestamps[i] = robotStartingTimeStamps + i * iterationPeriodSeconds
    }
}