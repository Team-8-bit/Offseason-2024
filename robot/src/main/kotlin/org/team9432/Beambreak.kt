package org.team9432

import org.team9432.lib.wrappers.beambreak.LoggedBeambreak

object Beambreak {
    val upperBeambreak = LoggedBeambreak(6, "Beambreaks/Upper")
    val lowerBeambreak = LoggedBeambreak(2, "Beambreaks/Lower")

    val hasNote get() = upperBeambreak.isTripped() || lowerBeambreak.isTripped()
    val hasNoNote get() = upperBeambreak.isClear() && lowerBeambreak.isClear()

    fun simClear() {
        upperBeambreak.setSimClear()
        lowerBeambreak.setSimClear()
    }
}