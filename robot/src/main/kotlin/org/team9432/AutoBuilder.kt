package org.team9432

import choreo.Choreo
import choreo.auto.AutoFactory
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import org.littletonrobotics.junction.Logger
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

    val shouldMirror = { Robot.alliance == DriverStation.Alliance.Red }
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
                Commands.runOnce({ drive.setPosition(traj.initialPose.get()) }).andThen(traj.cmd())
                    .andThen(autoCleanup())
            )

        return loop.cmd()
    }

    enum class CenterNote(val choreoName: String) {
        ONE("C1"), TWO("C2"), THREE("C3"), FOUR("C4"), FIVE("C5")
    }

    fun farsideTriple(): Command {
        val loop = factory.newLoop("FarsideTriple")

        val trajName = "Farside_Triple"
        val preload = factory.trajectory(trajName, 0, loop)
        val c5 = factory.trajectory(trajName, 1, loop)
        val c4 = factory.trajectory(trajName, 2, loop)
        val c3 = factory.trajectory(trajName, 3, loop)

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        loop.enabled().onTrue(
            Commands.runOnce({
                whenSimulated { simulatedPoseConsumer?.accept(preload.initialPose.get()) }
                drive.setPosition(preload.initialPose.get())
            }).andThen(
                ScheduleCommand(preload.cmd())
            )
        )

        preload.done().onTrue(aimAndScore().andThen(ScheduleCommand(c5.cmd())))

        c5.active().whileTrue(intake())
        c5.done().onTrue(aimAndScore().andThen(ScheduleCommand(c4.cmd())))

        c4.active().whileTrue(intake())
        c4.done().onTrue(aimAndScore().andThen(ScheduleCommand(c3.cmd())))

        c3.active().whileTrue(intake())
        c3.done().onTrue(aimAndScore().andThen(autoCleanup()))

        return loop.cmd()
    }

    fun ampsideTriple(): Command {
        val loop = factory.newLoop("AmpsideTriple")

        val trajName = "Ampside_Triple"
        val preload = factory.trajectory(trajName, 0, loop)
        val c1 = factory.trajectory(trajName, 1, loop)
        val c2 = factory.trajectory(trajName, 2, loop)
        val c3 = factory.trajectory(trajName, 3, loop)

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        loop.enabled().onTrue(
            Commands.runOnce({
                whenSimulated { simulatedPoseConsumer?.accept(preload.initialPose.get()) }
                drive.setPosition(preload.initialPose.get())
            }).andThen(
                ScheduleCommand(preload.cmd())
            )
        )

        preload.done().onTrue(aimAndScore().andThen(ScheduleCommand(c1.cmd())))

        c1.active().whileTrue(intake())
        c1.done().onTrue(aimAndScore().andThen(ScheduleCommand(c2.cmd())))

        c2.active().whileTrue(intake())
        c2.done().onTrue(aimAndScore().andThen(ScheduleCommand(c3.cmd())))

        c3.active().whileTrue(intake())
        c3.done().onTrue(aimAndScore().andThen(autoCleanup()))

        return loop.cmd()
    }

    fun fourClose(): Command {
        val loop = factory.newLoop("FourClose")

        val trajName = "Four_Close"
        val preload = factory.trajectory(trajName, 0, loop)
        val s1 = factory.trajectory(trajName, 1, loop)
        val s2 = factory.trajectory(trajName, 2, loop)
        val s3 = factory.trajectory(trajName, 3, loop)

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        loop.enabled().onTrue(
            Commands.runOnce({
                whenSimulated { simulatedPoseConsumer?.accept(preload.initialPose.get()) }
                drive.setPosition(preload.initialPose.get())
            }).andThen(
                ScheduleCommand(preload.cmd())
            )
        )

        preload.done().onTrue(aimAndScore().andThen(ScheduleCommand(s1.cmd())))

        s1.active().whileTrue(intake())
        s1.done().onTrue(aimAndScore().andThen(ScheduleCommand(s2.cmd())))

        s2.active().whileTrue(intake())
        s2.done().onTrue(aimAndScore().andThen(ScheduleCommand(s3.cmd())))

        s3.active().whileTrue(intake())
        s3.done().onTrue(aimAndScore().andThen(autoCleanup()))

        return loop.cmd()
    }

    fun smartFarsideTriple(noteOrder: Set<CenterNote>): Command {
        val noteQueue: Queue<CenterNote> = LinkedList(noteOrder)

        val loop = factory.newLoop("SmartFarsideTriple")

        val AMPtoAMPSHOT = factory.trajectory("AMPtoAMPSHOT", loop)
        val AMPSHOTtoC1 = factory.trajectory("AMPSHOTtoC1", loop)
        val AMPSHOTtoC2 = factory.trajectory("AMPSHOTtoC2", loop)
        val AMPSHOTtoC3 = factory.trajectory("AMPSHOTtoC3", loop)

        val C1toAMPSHOT = factory.trajectory("C1toAMPSHOT", loop)
        val C2toAMPSHOT = factory.trajectory("C2toAMPSHOT", loop)
        val C3toAMPSHOT = factory.trajectory("C3toAMPSHOT", loop)

        val C1toC2 = factory.trajectory("C1toC2", loop)
        val C1toC3 = factory.trajectory("C1toC3", loop)
        val C2toC1 = factory.trajectory("C2toC1", loop)
        val C2toC3 = factory.trajectory("C2toC3", loop)
        val C3toC1 = factory.trajectory("C3toC1", loop)
        val C3toC2 = factory.trajectory("C3toC2", loop)

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        loop.enabled().onTrue(
            Commands.runOnce({
                whenSimulated { simulatedPoseConsumer?.accept(AMPtoAMPSHOT.initialPose.get()) }
                drive.setPosition(AMPtoAMPSHOT.initialPose.get())
            }).andThen(AMPtoAMPSHOT.cmd())
        )

        // Tracking whether the robot has a note
        var touchedNote = false
        rollers.noteCurrentTrigger.onTrue(Commands.runOnce({ touchedNote = true }))

        val hasNoteSupplier = { touchedNote || Beambreak.hasNote }
        val hasNoNoteSupplier = { !touchedNote && Beambreak.hasNoNote }

        // True if there are more notes to grab
        val moreTargetNotes = { noteQueue.isNotEmpty() }

        // Intake while driving to centerline
        AMPSHOTtoC1.active().onTrue(intake())
        AMPSHOTtoC2.active().onTrue(intake())
        AMPSHOTtoC3.active().onTrue(intake())

        // Triggers for arriving at a center note from any other path
        val arrivingAtC1 = AMPSHOTtoC1.done()
            .or(C2toC1.done())
            .or(C3toC1.done())
        val arrivingAtC2 = AMPSHOTtoC2.done()
            .or(C1toC2.done())
            .or(C3toC2.done())
        val arrivingAtC3 = AMPSHOTtoC3.done()
            .or(C1toC3.done())
            .or(C2toC3.done())

        // After getting to centerline, return if note
        arrivingAtC1
            .and(hasNoteSupplier)
            .onTrue(C1toAMPSHOT.cmd())
        arrivingAtC2
            .and(hasNoteSupplier)
            .onTrue(C2toAMPSHOT.cmd())
        arrivingAtC3
            .and(hasNoteSupplier)
            .onTrue(C3toAMPSHOT.cmd())

        // After getting to centerline, keep looking if no note
        arrivingAtC1
            .and(hasNoNoteSupplier)
            .and(moreTargetNotes)
            .onTrue(Commands.defer({
                when (noteQueue.poll()) {
                    CenterNote.TWO -> C1toC2.cmd()
                    CenterNote.THREE -> C1toC3.cmd()
                    else -> {
                        loop.kill()
                        DriverStation.reportError("Error in SmartFarsideTriple auto, invalid note!", true)
                        Commands.none()
                    }
                }
            }, setOf(drive)))
        arrivingAtC2
            .and(hasNoNoteSupplier)
            .and(moreTargetNotes)
            .onTrue(Commands.defer({
                when (noteQueue.poll()) {
                    CenterNote.ONE -> C2toC1.cmd()
                    CenterNote.THREE -> C2toC3.cmd()
                    else -> {
                        loop.kill()
                        DriverStation.reportError("Error in SmartFarsideTriple auto, invalid note!", true)
                        Commands.none()
                    }
                }
            }, setOf(drive)))
        arrivingAtC3
            .and(hasNoNoteSupplier)
            .and(moreTargetNotes)
            .onTrue(Commands.defer({
                when (noteQueue.poll()) {
                    CenterNote.ONE -> C3toC1.cmd()
                    CenterNote.TWO -> C3toC2.cmd()
                    else -> {
                        loop.kill()
                        DriverStation.reportError("Error in SmartFarsideTriple auto, invalid note!", true)
                        Commands.none()
                    }
                }
            }, setOf(drive)))


        val arrivingAtAMPSHOT =
            C1toAMPSHOT.done()
                .or(C2toAMPSHOT.done())
                .or(C3toAMPSHOT.done())
                .or(AMPtoAMPSHOT.done())

        // After arriving to score, shoot
        arrivingAtAMPSHOT.onTrue(aimAndScore().andThen(Commands.runOnce({ touchedNote = false })))

        arrivingAtAMPSHOT
            .and(moreTargetNotes)
            .onTrue(
                Commands.waitUntil(hasNoNoteSupplier).andThen(
                    Commands.defer({
                        when (noteQueue.poll()) {
                            CenterNote.ONE -> AMPSHOTtoC1.cmd()
                            CenterNote.TWO -> AMPSHOTtoC2.cmd()
                            CenterNote.THREE -> AMPSHOTtoC3.cmd()
                            else -> {
                                loop.kill()
                                DriverStation.reportError("Error in SmartFarsideTriple auto, invalid note!", true)
                                Commands.none()
                            }
                        }
                    }, setOf(drive))
                )
            )

        return loop.cmd()
    }

    private fun autoCleanup(): Command = Commands.runOnce({ drive.clearTrajectoryInput() })

    fun aimAndScore() = Commands.parallel(
        drive.aimSpeaker(),
        pivot.runGoal(Pivot.Goal.SPEAKER_AIM),
        Commands.waitUntil(pivot::atGoal).andThen(
            Commands.runOnce({ noteSimulation?.animateShoot() }),
            rollers.runGoal(Rollers.Goal.SHOOTER_FEED),
        )
    )
        .withTimeout(1.0)
        .withName("Auto Aim and Score")
        .finallyDo { _ -> Beambreak.simClear() } // Remove note from the robot in sim


    // To add to AutoCommands
    fun Drive.aimSpeaker() = Commands.startEnd(
        {
            clearTrajectoryInput()
            setAutoAimGoal({ RobotState.getStandardAimingParameters().drivetrainAngle }, { 0.3 })
        },
        { clearAutoAimGoal() }
    )

    fun intake() =
        Commands.parallel(
            pivot.runGoal(Pivot.Goal.INTAKE),
            Commands.waitUntil(pivot::atGoal).andThen(
                rollers.runGoal(Rollers.Goal.INTAKE)
                    .until(Beambreak::hasNote).afterSimCondition({ noteSimulation!!.hasNote }, { Beambreak.lowerBeambreak.setSimTripped() })
                    .andThen(ScheduleCommand(alignNote()))
            )
        )
            .withName("AutoIntake")

    private fun alignNote() = Commands.sequence(
        Commands.runOnce({ noteSimulation?.animateAlign() }),
        rollers.runGoal(Rollers.Goal.ALIGN_FORWARD).until(Beambreak.upperBeambreak::isTripped).afterSimDelay(0.2) { Beambreak.upperBeambreak.setSimTripped() },
    )
        .alongWith(pivot.runGoal(Pivot.Goal.SPEAKER_AIM))
        .withTimeout(3.0)
        .withName("Note Align")
}