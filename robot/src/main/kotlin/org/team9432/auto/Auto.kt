package org.team9432.auto

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory


object ChoreoTrajectories {
    val fourAndNothing: ArrayList<ChoreoTrajectory> = Choreo.getTrajectoryGroup("4AndNothing")
    val fourAndCenter: ArrayList<ChoreoTrajectory> = Choreo.getTrajectoryGroup("4AndCenter")
}

enum class Auto(val prettyName: String, val auto: suspend () -> Unit) {
    FOUR_AND_NOTHING("Four and Nothing", FourNote::runFourNote),
    FOUR_AND_CENTER("Four and Center", FourNote::runFourNoteCenter),
    NOTHING("Do Nothing", {})
}
