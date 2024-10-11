//package org.team9432.auto
//
//object SharedRobotAuto {
////    suspend fun preload(firstPath: ChoreoTrajectory) {
////        //Swerve.resetOdometry(firstPath.getAutoFlippedInitialPose())
////        whenSimulated { Drive.setActualSimPose(firstPath.getAutoFlippedInitialPose()) }
////
////        Shooter.setState(Shooter.State.VISION_SHOOT)
////        parallel(
////            { Drive.followChoreo(firstPath) },
////            { simDelay(1.seconds) }
////        )
////
////        if (Beambreaks.hasNote) {
////            delay(0.1.seconds)
////            Actions.visionShoot(spindown = false)
////        }
////    }
//
////    suspend fun scoreNote(note: ChoreoTrajectory) = coroutineScope {
////        val intakingJob = launch { Actions.intake() }
////        Drive.followChoreo(note)
////
////        if (!Beambreaks.hasNote) {
////            intakingJob.cancelAndJoin()
////            delay(0.5.seconds)
////        } else if (Beambreaks.hasNote) {
////            delay(0.1.seconds)
////            Actions.visionShoot(spindown = false)
////        }
////    }
//}