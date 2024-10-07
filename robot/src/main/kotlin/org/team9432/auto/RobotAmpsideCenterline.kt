package org.team9432.auto

import com.choreo.lib.ChoreoTrajectory
import org.team9432.auto.SharedRobotAuto.preload
import org.team9432.auto.SharedRobotAuto.scoreNote
import org.team9432.auto.paths.AmpsideCenterlinePaths
import org.team9432.auto.types.AmpsideCenterline
import org.team9432.lib.util.ChoreoUtil
import java.util.*

object RobotAmpsideCenterline {
    suspend fun run(auto: AmpsideCenterline) {
        val trajectories: Queue<ChoreoTrajectory> = LinkedList(AmpsideCenterlinePaths.getSegmentsFor(auto).map { ChoreoUtil.getTrajectoryWithCache(it.name) })

        val preload = trajectories.poll()
        preload(preload)

        if (auto.scoreCloseAmpNote) {
            val path = trajectories.poll()
            scoreNote(path)
        }

        for (path in trajectories) {
            scoreNote(path)
        }

//        Shooter.setState(Shooter.State.IDLE)
    }
}