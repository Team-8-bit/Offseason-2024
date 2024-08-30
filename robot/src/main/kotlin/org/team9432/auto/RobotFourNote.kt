package org.team9432.auto

import com.choreo.lib.ChoreoTrajectory
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.Actions
import org.team9432.Beambreaks
import org.team9432.auto.SharedRobotAuto.preload
import org.team9432.auto.SharedRobotAuto.scoreNote
import org.team9432.auto.paths.FourNotePaths
import org.team9432.auto.types.FourNote
import org.team9432.auto.types.FourNote.EndAction.*
import org.team9432.lib.coroutines.parallel
import org.team9432.lib.util.ChoreoUtil
import org.team9432.lib.util.ChoreoUtil.getAutoFlippedInitialPose
import org.team9432.lib.util.simDelay
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration.Companion.seconds

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
                Shooter.setState(Shooter.State.IDLE)
                driveToCenterEnd(driveToCenter)
            }
        }

        Shooter.setState(Shooter.State.IDLE)
    }

    private suspend fun driveToCenterEnd(driveToCenter: ChoreoTrajectory) {
        Swerve.followChoreo(driveToCenter)
    }
}