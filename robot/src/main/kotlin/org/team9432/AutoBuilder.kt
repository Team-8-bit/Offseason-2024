package org.team9432

import choreo.Choreo
import choreo.auto.AutoFactory
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger
import org.team9432.lib.Library
import org.team9432.lib.util.afterSimCondition
import org.team9432.lib.util.afterSimDelay
import org.team9432.lib.util.whenSimulated
import org.team9432.resources.drive.Drive
import org.team9432.resources.drive.controllers.ChoreoTrajectoryController
import org.team9432.resources.flywheels.Flywheels
import org.team9432.resources.pivot.Pivot
import org.team9432.resources.rollers.Rollers
import java.util.*
import java.util.function.Consumer

class AutoBuilder(
    private val drive: Drive,
    private val pivot: Pivot,
    private val rollers: Rollers,
    private val flywheels: Flywheels,
    private val noteSimulation: NoteSimulation?,
    private val simulatedPoseConsumer: Consumer<Pose2d>?,
) {
    private val controller = ChoreoTrajectoryController(drive) { drive.currentTrajectoryModuleForces = it }

    val shouldMirror = { Library.alliance == DriverStation.Alliance.Red }
    private val factory = AutoFactory(
        RobotState::currentPose,
        { pose, sample -> controller.calculate(pose, sample as SwerveSample) },
        shouldMirror,
        drive,
        AutoFactory.AutoBindings(),
        Optional.of(TrajectoryLogger())
    )

    private inner class TrajectoryLogger: Choreo.TrajectoryLogger<SwerveSample> {
        override fun accept(trajectory: Trajectory<SwerveSample>, starting: Boolean) {
            Logger.recordOutput("Drive/Trajectory/Poses", *trajectory.poses)
            Logger.recordOutput("Drive/Trajectory/Start", trajectory.getInitialPose(shouldMirror()))
            Logger.recordOutput("Drive/Trajectory/End", trajectory.getFinalPose(shouldMirror()))
            Logger.recordOutput("Drive/Trajectory/Running", starting)
            Logger.recordOutput("Drive/Trajectory/Name", trajectory.name())
            Logger.recordOutput("Drive/Trajectory/TheoreticalTime", trajectory.totalTime)
        }
    }

    fun test(): Command {
        val loop = factory.newLoop("Test Auto")
        val traj = factory.trajectory("Test Path", loop)

        loop.enabled()
            .onTrue(
                Commands.runOnce({ drive.setPosition(traj.initialPose.get()) }).andThen(traj.cmdWithEnd())
                    .andThen(Commands.runOnce({ drive.acceptTrajectoryInput(ChassisSpeeds()) }))
            )

        return loop.cmd()
    }

    enum class CenterNote(val choreoName: String) {
        ONE("C1"), TWO("C2"), THREE("C3"), FOUR("C4"), FIVE("C5")
    }

    fun farsideTriple(): Command {
        val loop = factory.newLoop("FarsideTriple")

        val trajName = "Farside_Triple"
        val preload = factory.trajectory(trajName, 0, loop).applyStopOnEnd()
        val c5 = factory.trajectory(trajName, 1, loop).applyStopOnEnd()
        val c4 = factory.trajectory(trajName, 2, loop).applyStopOnEnd()
        val c3 = factory.trajectory(trajName, 3, loop).applyStopOnEnd()

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        loop.enabled().onTrue(
            Commands.runOnce({
                whenSimulated { simulatedPoseConsumer?.accept(preload.initialPose.get()) }
                drive.setPosition(preload.initialPose.get())
            }).andThen(
                ScheduleCommand(preload.cmdWithEnd())
            )
        )

        preload.done().onTrue(aimAndScore().andThen(ScheduleCommand(c5.cmdWithEnd())))

        c5.active().whileTrue(intake())
        c5.done().onTrue(aimAndScore().andThen(ScheduleCommand(c4.cmdWithEnd())))

        c4.active().whileTrue(intake())
        c4.done().onTrue(aimAndScore().andThen(ScheduleCommand(c3.cmdWithEnd())))

        c3.active().whileTrue(intake())
        c3.done().onTrue(aimAndScore().andThen(Commands.runOnce({ drive.acceptTrajectoryInput(ChassisSpeeds()) })))

        return loop.cmd()
    }

    fun ampsideTriple(): Command {
        val loop = factory.newLoop("AmpsideTriple")

        val trajName = "Ampside_Triple"
        val preload = factory.trajectory(trajName, 0, loop).applyStopOnEnd()
        val c1 = factory.trajectory(trajName, 1, loop).applyStopOnEnd()
        val c2 = factory.trajectory(trajName, 2, loop).applyStopOnEnd()
        val c3 = factory.trajectory(trajName, 3, loop).applyStopOnEnd()

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        loop.enabled().onTrue(
            Commands.runOnce({
                whenSimulated { simulatedPoseConsumer?.accept(preload.initialPose.get()) }
                drive.setPosition(preload.initialPose.get())
            }).andThen(
                ScheduleCommand(preload.cmdWithEnd())
            )
        )

        preload.done().onTrue(aimAndScore().andThen(ScheduleCommand(c1.cmdWithEnd())))

        c1.active().whileTrue(intake())
        c1.done().onTrue(aimAndScore().andThen(ScheduleCommand(c2.cmdWithEnd())))

        c2.active().whileTrue(intake())
        c2.done().onTrue(aimAndScore().andThen(ScheduleCommand(c3.cmdWithEnd())))

        c3.active().whileTrue(intake())
        c3.done().onTrue(aimAndScore().andThen(Commands.runOnce({ drive.acceptTrajectoryInput(ChassisSpeeds()) })))

        return loop.cmd()
    }

    private fun AutoTrajectory.applyStopOnEnd(): AutoTrajectory {
        done().onTrue(Commands.runOnce({ controller.stopAndReset() }))
        return this
    }

    fun ampSpikeFarsideCenterline(noteOrder: Set<CenterNote>): Command {
        val loop = factory.newLoop("FarsideCenterlineAmpSpikeStart")

        val preload = factory.trajectory("AMPtoAMPSHOTCLOSE", 0, loop)
        val toAMPSHOTCLOSE = factory.trajectory("AMPtoAMPSHOTCLOSE", 1, loop)

        loop.enabled().onTrue(
            Commands.runOnce({
                noteSimulation?.addPreload()
                whenSimulated { simulatedPoseConsumer?.accept(preload.initialPose.get()) }
                drive.setPosition(preload.initialPose.get())
            }).andThen(preload.cmdWithEnd())
        )

        // Prepare to shoot while driving
        preload.active().whileTrue(pivot.runGoal(Pivot.Goal.SPEAKER_AIM))

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        // After arriving to score, shoot
        preload.active().onFalse(aimAndScore())
        preload.active().onFalse(Commands.waitUntil(Beambreak::hasNoNote).andThen(toAMPSHOTCLOSE.cmdWithEnd()))

        toAMPSHOTCLOSE.active().whileTrue(intake())
        toAMPSHOTCLOSE.active().onFalse(aimAndScore())
        toAMPSHOTCLOSE.active()
            .onFalse(
                Commands.waitUntil(Beambreak::hasNoNote).andThen(
                    farsideCenterlinePortion(startPosition = FarsideCenterlinePortionStart.AMPSHOT_CLOSE, noteOrder)
                ).onlyIf(noteOrder::isNotEmpty)
            )

        return loop.cmd()
    }

    fun ogFarsideCenterline(noteOrder: Set<CenterNote>): Command {
        val loop = factory.newLoop("FarsideCenterlineSkipSpikeStart")

        val AMPtoAMPSHOTFAR = factory.trajectory("AMPtoAMPSHOTFAR", loop)

        loop.enabled().onTrue(
            Commands.runOnce({
                noteSimulation?.addPreload()
                whenSimulated { simulatedPoseConsumer?.accept(AMPtoAMPSHOTFAR.initialPose.get()) }
                drive.setPosition(AMPtoAMPSHOTFAR.initialPose.get())
            }).andThen(AMPtoAMPSHOTFAR.cmdWithEnd())
        )

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        // Prepare to shoot while driving
        AMPtoAMPSHOTFAR.active().whileTrue(pivot.runGoal(Pivot.Goal.SPEAKER_AIM))

        // After arriving to score, shoot
        AMPtoAMPSHOTFAR.active().onFalse(aimAndScore())
        AMPtoAMPSHOTFAR.active()
            .onFalse(
                Commands.waitUntil(Beambreak::hasNoNote).andThen(
                    farsideCenterlinePortion(startPosition = FarsideCenterlinePortionStart.AMPSHOT_FAR, noteOrder)
                ).onlyIf(noteOrder::isNotEmpty)
            )

        return loop.cmd()
    }

    enum class FarsideCenterlinePortionStart {
        AMPSHOT_CLOSE, AMPSHOT_FAR
    }

    fun farsideCenterlinePortion(startPosition: FarsideCenterlinePortionStart, noteOrder: Set<CenterNote>): Command {
        val noteQueue: Queue<CenterNote> = LinkedList(noteOrder)

        val loop = factory.newLoop("FarsideCenterlinePortion")

        // True if there are more notes to grab
        val moreTargetNotes = { noteQueue.isNotEmpty() }

        val AMPSHOTFARtoC1 = factory.trajectory("AMPSHOTFARtoC1", loop)
        val AMPSHOTFARtoC2 = factory.trajectory("AMPSHOTFARtoC2", loop)
        val AMPSHOTFARtoC3 = factory.trajectory("AMPSHOTFARtoC3", loop)

        val AMPSHOTCLOSEtoC1 = factory.trajectory("AMPSHOTCLOSEtoC1", loop)
        val AMPSHOTCLOSEtoC2 = factory.trajectory("AMPSHOTCLOSEtoC2", loop)
        val AMPSHOTCLOSEtoC3 = factory.trajectory("AMPSHOTCLOSEtoC3", loop)

        val C1toAMPSHOTFAR = factory.trajectory("C1toAMPSHOTFAR", loop)
        val C2toAMPSHOTFAR = factory.trajectory("C2toAMPSHOTFAR", loop)
        val C3toAMPSHOTFAR = factory.trajectory("C3toAMPSHOTFAR", loop)

        val C1toC2 = factory.trajectory("C1toC2", loop)
        val C1toC3 = factory.trajectory("C1toC3", loop)
        val C2toC1 = factory.trajectory("C2toC1", loop)
        val C2toC3 = factory.trajectory("C2toC3", loop)
        val C3toC1 = factory.trajectory("C3toC1", loop)
        val C3toC2 = factory.trajectory("C3toC2", loop)

        fun exit(): Command {
            loop.kill()
            DriverStation.reportError("Error in SmartFarsideTriple auto, invalid note!", true)
            return Commands.none()
        }

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        loop.enabled()
            .and(moreTargetNotes)
            .onTrue(
                Commands.defer({
                    val firstNote = noteQueue.poll()
                    when (startPosition) {
                        FarsideCenterlinePortionStart.AMPSHOT_CLOSE -> when (firstNote) {
                            CenterNote.ONE -> AMPSHOTCLOSEtoC1.cmdWithEnd()
                            CenterNote.TWO -> AMPSHOTCLOSEtoC2.cmdWithEnd()
                            CenterNote.THREE -> AMPSHOTCLOSEtoC3.cmdWithEnd()
                            else -> exit()
                        }

                        FarsideCenterlinePortionStart.AMPSHOT_FAR -> when (firstNote) {
                            CenterNote.ONE -> AMPSHOTFARtoC1.cmdWithEnd()
                            CenterNote.TWO -> AMPSHOTFARtoC2.cmdWithEnd()
                            CenterNote.THREE -> AMPSHOTFARtoC3.cmdWithEnd()
                            else -> exit()
                        }
                    }
                }, setOf(drive))
            )

        // Intake while driving to centerline
        AMPSHOTFARtoC1.active().onTrue(intake())
        AMPSHOTFARtoC2.active().onTrue(intake())
        AMPSHOTFARtoC3.active().onTrue(intake())
        AMPSHOTCLOSEtoC1.active().onTrue(intake())
        AMPSHOTCLOSEtoC2.active().onTrue(intake())
        AMPSHOTCLOSEtoC3.active().onTrue(intake())

        // Triggers for arriving at a center note from any other path
        val arrivingAtC1 = AMPSHOTCLOSEtoC1.done()
            .or(AMPSHOTFARtoC1.done())
            .or(C2toC1.done())
            .or(C3toC1.done())
        val arrivingAtC2 = AMPSHOTCLOSEtoC2.done()
            .or(AMPSHOTFARtoC2.done())
            .or(C1toC2.done())
            .or(C3toC2.done())
        val arrivingAtC3 = AMPSHOTCLOSEtoC3.done()
            .or(AMPSHOTFARtoC3.done())
            .or(C1toC3.done())
            .or(C2toC3.done())

        // After getting to centerline, return if note
        arrivingAtC1.onTrue(
            Commands.either(
                C1toAMPSHOTFAR.cmdWithEnd(),
                ScheduleCommand(Commands.defer({
                    when (noteQueue.poll()) {
                        CenterNote.TWO -> C1toC2.cmdWithEnd()
                        CenterNote.THREE -> C1toC3.cmdWithEnd()
                        else -> exit()
                    }
                }, setOf(drive))).onlyIf({ moreTargetNotes.invoke() }),
                Beambreak::hasNote
            )
        )
        arrivingAtC2.onTrue(
            Commands.either(
                C2toAMPSHOTFAR.cmdWithEnd(),
                ScheduleCommand(Commands.defer({
                    when (noteQueue.poll()) {
                        CenterNote.ONE -> C2toC1.cmdWithEnd()
                        CenterNote.THREE -> C2toC3.cmdWithEnd()
                        else -> exit()
                    }
                }, setOf(drive))).onlyIf({ moreTargetNotes.invoke() }),
                Beambreak::hasNote
            )
        )
        arrivingAtC3.onTrue(
            Commands.either(
                C3toAMPSHOTFAR.cmdWithEnd(),
                ScheduleCommand(Commands.defer({
                    when (noteQueue.poll()) {
                        CenterNote.ONE -> C3toC1.cmdWithEnd()
                        CenterNote.TWO -> C3toC2.cmdWithEnd()
                        else -> exit()
                    }
                }, setOf(drive))).onlyIf({ moreTargetNotes.invoke() }),
                Beambreak::hasNote
            )
        )

        val drivingToAMPSHOTFAR =
            C1toAMPSHOTFAR.active()
                .or(C2toAMPSHOTFAR.active())
                .or(C3toAMPSHOTFAR.active())

        // Prepare to shoot while driving
        drivingToAMPSHOTFAR.whileTrue(pivot.runGoal(Pivot.Goal.SPEAKER_AIM))

        // After arriving to score, shoot
        drivingToAMPSHOTFAR
            .onFalse(aimAndScore())

        // If there are more notes to get, start another path
        drivingToAMPSHOTFAR
            .and(moreTargetNotes)
            .onFalse(
                // Wait until the notes is out of the robot and then start the next path
                Commands.waitUntil(Beambreak::hasNoNote).andThen(
                    Commands.defer({
                        when (noteQueue.poll()) {
                            CenterNote.ONE -> AMPSHOTFARtoC1.cmdWithEnd()
                            CenterNote.TWO -> AMPSHOTFARtoC2.cmdWithEnd()
                            CenterNote.THREE -> AMPSHOTFARtoC3.cmdWithEnd()
                            else -> exit()
                        }
                    }, setOf(drive))
                )
            )

        return loop.cmd()
    }

    fun AutoTrajectory.cmdWithEnd(): Command = cmd().finallyDo { _ -> controller.stopAndReset() }

    fun fourClose(): Command {
        val loop = factory.newLoop("FourClose")

        val trajName = "Four_Close"
        val preload = factory.trajectory(trajName, 0, loop).applyStopOnEnd()
        val s1 = factory.trajectory(trajName, 1, loop).applyStopOnEnd()
        val s2 = factory.trajectory(trajName, 2, loop).applyStopOnEnd()
        val s3 = factory.trajectory(trajName, 3, loop).applyStopOnEnd()

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        loop.enabled().onTrue(
            Commands.runOnce({
                noteSimulation?.addPreload()
                whenSimulated { simulatedPoseConsumer?.accept(preload.initialPose.get()) }
                drive.setPosition(preload.initialPose.get())
            }).andThen(preload.cmdWithEnd())
        )

        preload.done().onTrue(aimAndScore())
        preload.done().onTrue(Commands.waitUntil(Beambreak::hasNoNote).andThen(s1.cmdWithEnd()))

        s1.active().whileTrue(intake())
        s1.done().onTrue(aimAndScore())
        s1.done().onTrue(Commands.waitUntil(Beambreak::hasNoNote).andThen(s2.cmdWithEnd()))

        s2.active().whileTrue(intake())
        s2.done().onTrue(aimAndScore())
        s2.done().onTrue(Commands.waitUntil(Beambreak::hasNoNote).andThen(s3.cmdWithEnd()))

        var isFinished = false

        s3.active().whileTrue(intake())
        s3.done().onTrue(aimAndScore().andThen(Commands.runOnce({
            drive.acceptTrajectoryInput(ChassisSpeeds())
            isFinished = true
        })))

        loop.enabled().whileTrue(Commands.runOnce({ println(readyToShoot.asBoolean) }).repeatedly())
        return loop.cmd { isFinished }
    }

    val readyToShoot = Trigger { pivot.atGoal && drive.atAutoAimGoal(1.0) && flywheels.atGoal }.debounce(0.3, Debouncer.DebounceType.kRising)

    fun aimAndScore() = Commands.parallel(
        drive.aimSpeaker(),
        pivot.runGoal(Pivot.Goal.SPEAKER_AIM),
        Commands.waitUntil(readyToShoot).andThen(
            Commands.runOnce({ noteSimulation?.animateShoot() }),
            rollers.runGoal(Rollers.Goal.SHOOTER_FEED).afterSimDelay(0.15) { Beambreak.simClear() },
        )
    )
        .until(Beambreak.upperBeambreak::isClear)
        .withTimeout(2.5)
        .withName("Auto Aim and Score")


    // To add to AutoCommands
    fun Drive.aimSpeaker() = Commands.startEnd(
        {
            clearTrajectoryInput()
            setAutoAimGoal({ RobotState.getStandardAimingParameters().drivetrainAngle }, { 0.3 })
        },
        {
            clearAutoAimGoal()
        }
    )

    fun intake() =
        Commands.parallel(
            pivot.runGoal(Pivot.Goal.INTAKE),
            Commands.waitUntil(pivot::atGoal).andThen(
                Commands.runOnce({ noteSimulation?.animateAlign() }),
                rollers.runGoal(Rollers.Goal.INTAKE).until(Beambreak.lowerBeambreak::isTripped).afterSimCondition({ noteSimulation!!.hasNote }) { Beambreak.lowerBeambreak.setSimTripped() },
                rollers.runGoal(Rollers.Goal.ALIGN_FORWARD_SLOW).until(Beambreak.upperBeambreak::isTripped).afterSimDelay(0.1) { Beambreak.upperBeambreak.setSimTripped() },
                rollers.runGoal(Rollers.Goal.ALIGN_REVERSE_SLOW).withTimeout(0.05),
            )
        ).withName("AutoIntake")

    fun farsideCenterline(scoreSpike: Boolean, noteOrder: Set<CenterNote>): Command {
        return if (scoreSpike) ampSpikeFarsideCenterline(noteOrder) else ogFarsideCenterline(noteOrder)
    }
}