package org.team9432.auto

import org.team9432.auto.SharedRobotAuto.preload
import org.team9432.auto.SharedRobotAuto.scoreNote
import org.team9432.auto.paths.FarsideCenterlinePaths
import org.team9432.auto.types.FarsideCenterline
import org.team9432.lib.util.ChoreoUtil

object RobotFarsideCenterline {
    suspend fun run(auto: FarsideCenterline) {
        val trajectories = FarsideCenterlinePaths.getSegmentsFor(auto).map { ChoreoUtil.getTrajectoryWithCache(it.name) }

        when (auto.notes.size) {
            1 -> {
                val (preload, firstNote) = trajectories
                preload(preload)
                scoreNote(firstNote)
            }

            2 -> {
                val (preload, firstNote, secondNote) = trajectories
                preload(preload)
                scoreNote(firstNote)
                scoreNote(secondNote)
            }
        }

//        Shooter.setState(Shooter.State.IDLE)
    }
}