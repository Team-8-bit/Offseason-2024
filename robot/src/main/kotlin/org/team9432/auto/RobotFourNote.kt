package org.team9432.auto

import com.choreo.lib.ChoreoTrajectory
import org.team9432.auto.SharedRobotAuto.preload
import org.team9432.auto.SharedRobotAuto.scoreNote
import org.team9432.auto.paths.FourNotePaths
import org.team9432.auto.types.FourNote
import org.team9432.auto.types.FourNote.EndAction.*
import org.team9432.lib.util.ChoreoUtil

object RobotFourNote {
    suspend fun run(auto: FourNote) {
        val trajectories = FourNotePaths.getSegmentsFor(auto).map { ChoreoUtil.getTrajectoryWithCache(it.name) }

        when (auto.endAction) {
            DO_NOTHING -> {
                val (preload, firstNote, secondNote, thirdNote) = trajectories
                preload(preload)
                scoreNote(firstNote)
                scoreNote(secondNote)
                scoreNote(thirdNote)
            }

            SCORE_CENTERLINE -> {
                val (preload, firstNote, secondNote, thirdNote, centerNote) = trajectories
                preload(preload)
                scoreNote(firstNote)
                scoreNote(secondNote)
                scoreNote(thirdNote)
                scoreNote(centerNote)
            }

            DRIVE_TO_CENTER -> {
                val (preload, firstNote, secondNote, thirdNote, driveToCenter) = trajectories
                preload(preload)
                scoreNote(firstNote)
                scoreNote(secondNote)
                scoreNote(thirdNote)
//                Shooter.setState(Shooter.State.IDLE)
                driveToCenterEnd(driveToCenter)
            }
        }

//        Shooter.setState(Shooter.State.IDLE)
    }

    private suspend fun driveToCenterEnd(driveToCenter: ChoreoTrajectory) {
//        Drive.followChoreo(driveToCenter)
    }
}