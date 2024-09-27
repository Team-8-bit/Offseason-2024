package org.team9432

import org.team9432.lib.wrappers.beambreak.LoggedBeambreak

object Beambreaks {
    val upper = LoggedBeambreak(7, "Beambreaks/Upper")
    val lower = LoggedBeambreak(6, "Beambreaks/Lower")

    val hasNote get() = upper.isTripped() || lower.isTripped()
    val hasNoNote get() = upper.isClear() && lower.isClear()
}