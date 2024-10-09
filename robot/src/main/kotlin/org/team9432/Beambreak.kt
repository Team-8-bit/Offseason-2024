package org.team9432

import org.team9432.lib.wrappers.beambreak.LoggedBeambreak

object Beambreak {
    val upperBeambreak = LoggedBeambreak(8, "UpperBeambreak")
    val lowerBeambreak = LoggedBeambreak(6, "LowerBeambreak")

    val hasNote get() = upperBeambreak.isTripped() || lowerBeambreak.isTripped()
    val hasNoNote get() = upperBeambreak.isClear() && lowerBeambreak.isClear()

    fun simClear() {
        upperBeambreak.setSimClear()
        lowerBeambreak.setSimClear()
    }
}