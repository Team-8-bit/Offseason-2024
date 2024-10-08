package org.team9432

import org.team9432.lib.wrappers.beambreak.LoggedBeambreak

object Beambreak {
    val beambreak = LoggedBeambreak(8, "Beambreak")

    val hasNote get() = beambreak.isTripped()
    val hasNoNote get() = beambreak.isClear()

    fun simClear() {
        beambreak.setSimClear()
    }
}