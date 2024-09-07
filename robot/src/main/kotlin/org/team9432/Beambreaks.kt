package org.team9432

import org.team9432.lib.Beambreak
import org.team9432.lib.RobotPeriodicManager

object Beambreaks {
    val upper = Beambreak(7)
    val lower = Beambreak(6)

    val hasNote get() = upper.isTripped() || lower.isTripped()
    val hasNoNote get() = upper.isClear() && lower.isClear()

    init {
        RobotPeriodicManager.startPeriodic { log() }
    }

    private fun log() {
//        Logger.recordOutput("Beambreaks/Upper", upper)
//        Logger.recordOutput("Beambreaks/Lower", lower)
    }
}