package org.team9432.auto

import com.choreo.lib.ChoreoTrajectory
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.Actions
import org.team9432.Beambreaks
import org.team9432.lib.coroutines.parallel
import org.team9432.lib.util.ChoreoUtil.getAutoFlippedInitialPose
import org.team9432.lib.util.simDelay
import org.team9432.lib.util.whenSimulated
import org.team9432.resources.shooter.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration.Companion.seconds

object SharedRobotAuto {
    suspend fun preload(firstPath: ChoreoTrajectory) {
        Swerve.resetOdometry(firstPath.getAutoFlippedInitialPose())
        whenSimulated { Swerve.setActualSimPose(firstPath.getAutoFlippedInitialPose()) }

        Shooter.setState(Shooter.State.VISION_SHOOT)
        parallel(
            { Swerve.followChoreo(firstPath) },
            { simDelay(1.seconds) }
        )

        if (Beambreaks.hasNote) {
            delay(0.1.seconds)
            Actions.visionShoot(spindown = false)
        }
    }

    suspend fun scoreNote(note: ChoreoTrajectory) = coroutineScope {
        val intakingJob = launch { Actions.intake() }
        Swerve.followChoreo(note)

        if (!Beambreaks.hasNote) {
            intakingJob.cancelAndJoin()
            delay(0.5.seconds)
        } else if (Beambreaks.hasNote) {
            delay(0.1.seconds)
            Actions.visionShoot(spindown = false)
        }
    }
}