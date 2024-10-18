package org.team9432

import choreo.Choreo
import choreo.auto.AutoFactory
import choreo.auto.AutoLoop
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

    enum class AmpsideCenterNote {
        ONE, TWO, THREE
    }

    fun farsideTriple(): Command {
        val loop = factory.newLoop("FarsideTriple")
        val trajName = "Farside_Triple"
        return genericSplitTripleAuto(trajName, loop)
    }

    fun fourClose(): Command {
        val loop = factory.newLoop("FourClose")
        val trajName = "Four_Close"
        return genericSplitTripleAuto(trajName, loop)
    }

    private fun genericSplitTripleAuto(trajName: String, loop: AutoLoop): Command {
        val preload = factory.trajectory(trajName, 0, loop)
        val firstNote = factory.trajectory(trajName, 1, loop)
        val secondNote = factory.trajectory(trajName, 2, loop)
        val thirdNote = factory.trajectory(trajName, 3, loop)

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        loop.enabled().onTrue(
            Commands.runOnce({
                noteSimulation?.addPreload()
                whenSimulated { simulatedPoseConsumer?.accept(preload.initialPose.get()) }
                drive.setPosition(preload.initialPose.get())
            }).andThen(preload.cmdWithEnd())
        )

        preload.active().onTrue(setPivotAimFromPose(preload.finalPose.get()))
        preload.active().whileTrue(preparePivot())
        preload.active().onFalse(aimAndScore(loop, 2.0))
        preload.active().onFalse(Commands.waitUntil(shotComplete).andThen(firstNote.cmdWithEnd()))

        firstNote.active().onTrue(setPivotAimFromPose(firstNote.finalPose.get()))
        firstNote.active().onTrue(intake().andThen(preparePivot()))
        firstNote.active().onFalse(aimAndScore(loop, 2.0))
        firstNote.active().onFalse(Commands.waitUntil(shotComplete).andThen(secondNote.cmdWithEnd()))

        secondNote.active().onTrue(setPivotAimFromPose(secondNote.finalPose.get()))
        secondNote.active().onTrue(intake().andThen(preparePivot()))
        secondNote.active().onFalse(aimAndScore(loop, 2.0))
        secondNote.active().onFalse(Commands.waitUntil(shotComplete).andThen(thirdNote.cmdWithEnd()))

        thirdNote.active().onTrue(setPivotAimFromPose(thirdNote.finalPose.get()))
        thirdNote.active().onTrue(intake().andThen(preparePivot()))
        thirdNote.active().onFalse(aimAndScore(loop, 2.0))

        return loop.cmd()
    }

    private fun ampSpikeFarsideCenterline(noteOrder: Set<AmpsideCenterNote>): Command {
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

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        // Prepare to shoot while driving
        preload.active().onTrue(setPivotAimFromPose(preload.finalPose.get()))
        preload.active().whileTrue(preparePivot())
        preload.active().onFalse(aimAndScore(loop, 0.5))
        preload.active().onFalse(Commands.waitUntil(shotComplete).andThen(toAMPSHOTCLOSE.cmdWithEnd()))

        toAMPSHOTCLOSE.active().onTrue(setPivotAimFromPose(toAMPSHOTCLOSE.finalPose.get()))
        toAMPSHOTCLOSE.active().whileTrue(intake().andThen(preparePivot()))
        toAMPSHOTCLOSE.active().onFalse(aimAndScore(loop, 0.5))
        toAMPSHOTCLOSE.active().onFalse(
            Commands.waitUntil(shotComplete).andThen(
                farsideCenterlinePortion(startPosition = FarsideCenterlinePortionStart.AMPSHOT_CLOSE, noteOrder)
            ).onlyIf(noteOrder::isNotEmpty)
        )

        return loop.cmd()
    }

    private fun farsideCenterlineSkipSpike(noteOrder: Set<AmpsideCenterNote>): Command {
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
        AMPtoAMPSHOTFAR.active().onTrue(setPivotAimFromPose(AMPtoAMPSHOTFAR.finalPose.get()))
        AMPtoAMPSHOTFAR.active().whileTrue(preparePivot())

        // After arriving to score, shoot
        AMPtoAMPSHOTFAR.active().onFalse(aimAndScore(loop))
        AMPtoAMPSHOTFAR.active()
            .onFalse(
                Commands.waitUntil(shotComplete).andThen(
                    farsideCenterlinePortion(startPosition = FarsideCenterlinePortionStart.AMPSHOT_FAR, noteOrder)
                ).onlyIf(noteOrder::isNotEmpty)
            )

        return loop.cmd()
    }

    enum class FarsideCenterlinePortionStart {
        AMPSHOT_CLOSE, AMPSHOT_FAR
    }

    private fun farsideCenterlinePortion(startPosition: FarsideCenterlinePortionStart, noteOrder: Set<AmpsideCenterNote>): Command {
        val noteQueue: Queue<AmpsideCenterNote> = LinkedList(noteOrder)

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
                            AmpsideCenterNote.ONE -> AMPSHOTCLOSEtoC1.cmdWithEnd()
                            AmpsideCenterNote.TWO -> AMPSHOTCLOSEtoC2.cmdWithEnd()
                            AmpsideCenterNote.THREE -> AMPSHOTCLOSEtoC3.cmdWithEnd()
                            else -> exit()
                        }

                        FarsideCenterlinePortionStart.AMPSHOT_FAR -> when (firstNote) {
                            AmpsideCenterNote.ONE -> AMPSHOTFARtoC1.cmdWithEnd()
                            AmpsideCenterNote.TWO -> AMPSHOTFARtoC2.cmdWithEnd()
                            AmpsideCenterNote.THREE -> AMPSHOTFARtoC3.cmdWithEnd()
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
                        AmpsideCenterNote.TWO -> C1toC2.cmdWithEnd()
                        AmpsideCenterNote.THREE -> C1toC3.cmdWithEnd()
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
                        AmpsideCenterNote.ONE -> C2toC1.cmdWithEnd()
                        AmpsideCenterNote.THREE -> C2toC3.cmdWithEnd()
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
                        AmpsideCenterNote.ONE -> C3toC1.cmdWithEnd()
                        AmpsideCenterNote.TWO -> C3toC2.cmdWithEnd()
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

        val AMPSHOTFAR_endPose = C1toAMPSHOTFAR.finalPose.get()
        drivingToAMPSHOTFAR.onTrue(setPivotAimFromPose(AMPSHOTFAR_endPose))
        // Prepare to shoot while driving
        drivingToAMPSHOTFAR.whileTrue(preparePivot())

        // After arriving to score, shoot
        drivingToAMPSHOTFAR
            .onFalse(aimAndScore(loop))

        // If there are more notes to get, start another path
        drivingToAMPSHOTFAR
            .and(moreTargetNotes)
            .onFalse(
                // Wait until the notes is out of the robot and then start the next path
                Commands.waitUntil(shotComplete).andThen(
                    Commands.defer({
                        when (noteQueue.poll()) {
                            AmpsideCenterNote.ONE -> AMPSHOTFARtoC1.cmdWithEnd()
                            AmpsideCenterNote.TWO -> AMPSHOTFARtoC2.cmdWithEnd()
                            AmpsideCenterNote.THREE -> AMPSHOTFARtoC3.cmdWithEnd()
                            else -> exit()
                        }
                    }, setOf(drive))
                )
            )

        return loop.cmd()
    }

    fun farsideCenterline(scoreSpike: Boolean, noteOrder: Set<AmpsideCenterNote>): Command {
        return if (scoreSpike) ampSpikeFarsideCenterline(noteOrder) else farsideCenterlineSkipSpike(noteOrder)
    }

    private fun readyToShoot(loop: AutoLoop, toleranceDegrees: Double) =
        Trigger(loop.loop) { pivot.atGoal && drive.atAutoAimGoal(toleranceDegrees) && flywheels.atGoal }//.debounce(0.3, Debouncer.DebounceType.kRising)

    private fun AutoTrajectory.cmdWithEnd(): Command = cmd().finallyDo { _ -> controller.stopAndReset() }

    private val shotComplete = Trigger(Beambreak::hasNoNote).debounce(0.1)

    private fun aimAndScore(loop: AutoLoop, toleranceDegrees: Double = 1.0) =
        Commands.parallel(
            drive.aimSpeaker(),
            pivot.runGoal(Pivot.Goal.SPEAKER_AIM),
            Commands.waitUntil(readyToShoot(loop, toleranceDegrees)).andThen(
                Commands.runOnce({ noteSimulation?.animateShoot() }),
                rollers.runGoal(Rollers.Goal.SHOOTER_FEED).afterSimDelay(0.15) { Beambreak.simClear() },
            )
        )
            .beforeStarting({ RobotState.autoPivotAimFromPose = null })
            .until(shotComplete)
            .withTimeout(2.5)
            .withName("Auto Aim and Score")

    private fun preparePivot() = pivot.runGoal(Pivot.Goal.SPEAKER_AIM)

    private fun setPivotAimFromPose(pose2d: Pose2d): Command = Commands.runOnce({ RobotState.autoPivotAimFromPose = pose2d })

    // To add to AutoCommands
    private fun Drive.aimSpeaker() = Commands.startEnd(
        {
            clearTrajectoryInput()
            setAutoAimGoal({ RobotState.getStandardAimingParameters().drivetrainAngle }, { 0.3 })
        },
        {
            clearAutoAimGoal()
        }
    )

    private fun intake() =
        Commands.waitUntil(pivot::atGoal).andThen(
            Commands.runOnce({ noteSimulation?.animateAlign() }),
            rollers.runGoal(Rollers.Goal.INTAKE).until(Beambreak.lowerBeambreak::isTripped).afterSimCondition({ noteSimulation!!.hasNote }) { Beambreak.lowerBeambreak.setSimTripped() },
            rollers.runGoal(Rollers.Goal.ALIGN_FORWARD_SLOW).until(Beambreak.upperBeambreak::isTripped).afterSimDelay(0.1) { Beambreak.upperBeambreak.setSimTripped() },
            rollers.runGoal(Rollers.Goal.ALIGN_FORWARD_SLOW).withTimeout(0.1),
            rollers.runGoal(Rollers.Goal.ALIGN_REVERSE_SLOW).withTimeout(0.05),
        )
            .deadlineWith(pivot.runGoal(Pivot.Goal.IDLE))
            .withName("AutoIntake")
}