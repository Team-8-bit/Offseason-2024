package org.team9432.auto

import com.choreo.lib.Choreo
import com.choreo.lib.ChoreoTrajectory


object ChoreoTrajectories {
    val fourAndNothing: ArrayList<ChoreoTrajectory> = Choreo.getTrajectoryGroup("4AndNothing")
    val reverseFourAndNothing: ArrayList<ChoreoTrajectory> = Choreo.getTrajectoryGroup("Reverse4AndNothing")
    val fourAndCenter: ArrayList<ChoreoTrajectory> = Choreo.getTrajectoryGroup("4AndCenter")
    val fourAndCenter1: ArrayList<ChoreoTrajectory> = Choreo.getTrajectoryGroup("4AndCenter1")
    val fourAndCenter2: ArrayList<ChoreoTrajectory> = Choreo.getTrajectoryGroup("4AndCenter2")
    val center54: ArrayList<ChoreoTrajectory> = Choreo.getTrajectoryGroup("Center54")
    val center12: ArrayList<ChoreoTrajectory> = Choreo.getTrajectoryGroup("Center12")
    val ampAndCenter12: ArrayList<ChoreoTrajectory> = Choreo.getTrajectoryGroup("AmpAndCenter12")
}

enum class Auto(val prettyName: String, val auto: suspend () -> Unit) {
    FOUR_AND_NOTHING("Four and Nothing", FourNote::runFourAndNothing),
    REVERSE_FOUR_AND_NOTHING("Reversed Four and Nothing", FourNote::runReverseFourAndNothing),
    FOUR_AND_CENTER("Four and Center", FourNote::runFourNoteCenter),
    FOUR_AND_CENTER_1("Four and Center 1", FourNote::runFourAndCenter1),
    FOUR_AND_CENTER_2("Four and Center 2", FourNote::runFourAndCenter2),
    CENTER_54("Center 54", FourNote::runCenter54),
    CENTER_12("Center 12", FourNote::runCenter12),
    AMP_AND_CENTER_12("Amp And Center 12", FourNote::runAmpAndCenter12),
    NOTHING("Do Nothing", {})
}
