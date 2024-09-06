package org.team9432.resources.swerve

import org.team9432.Robot
import org.team9432.annotation.Logged
import org.team9432.lib.coroutines.Team8BitRobot
import org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils.Simulations.SwerveDriveSimulation

interface OdometryThread {
    @Logged
    open class OdometryThreadInputs {
        var measurementTimestamps: DoubleArray = DoubleArray(0)
    }

    fun updateInputs(inputs: OdometryThreadInputs) {}

    fun start() {}

    companion object {
        fun createInstance(): OdometryThread {
            return when (Robot.runtime) {
                Team8BitRobot.Runtime.REAL -> OdometryThreadReal
                Team8BitRobot.Runtime.SIM -> SwerveDriveSimulation.OdometryThreadSim()
                Team8BitRobot.Runtime.REPLAY -> object: OdometryThread {}
            }
        }
    }
}