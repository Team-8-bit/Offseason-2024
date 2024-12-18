package org.team9432.resources.drive.odometrythread

import org.team9432.Robot
import org.team9432.annotation.Logged
import org.team9432.lib.Library.Runtime

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
                Runtime.REAL -> OdometryThreadReal
                Runtime.SIM -> OdometryThreadSim()
                Runtime.REPLAY -> object: OdometryThread {}
            }
        }
    }
}