package org.team9432

import org.team9432.lib.Beambreak
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.doglog.Logger

object Beambreaks {
    val upper = Beambreak(7)
    val lower = Beambreak(6)

    val hasNote get() = upper.isTripped() || lower.isTripped()

    init {
        RobotPeriodicManager.startPeriodic { log() }
    }

    private fun log() {
        Logger.log("Beambreaks/Upper", upper)
        Logger.log("Beambreaks/Lower", lower)
    }
}