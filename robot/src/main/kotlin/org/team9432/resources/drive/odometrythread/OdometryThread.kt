package org.team9432.resources.drive.odometrythread

import org.team9432.Robot
import org.team9432.annotation.Logged
import org.team9432.lib.coroutines.Team8BitRobot

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
                Team8BitRobot.Runtime.SIM -> OdometryThreadSim()
                Team8BitRobot.Runtime.REPLAY -> object: OdometryThread {}
            }
        }
    }
}