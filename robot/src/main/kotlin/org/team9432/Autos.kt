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

class Autos(
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

    private fun autoCleanup(): Command = Commands.runOnce({ drive.clearTrajectoryInput() })

    fun aimAndScore() = Commands.parallel(
        drive.aimSpeaker(),
        pivot.runGoal(Pivot.Goal.SPEAKER_AIM),
        Commands.waitUntil(pivot::atGoal).andThen(
            Commands.runOnce({ noteSimulation?.animateShoot() }),
            rollers.runGoal(Rollers.Goal.SHOOTER_FEED),
        )
    )
        .withTimeout(0.5)
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
                    .andThen(alignNote())
            )
        )
            .withName("AutoIntake")

    private fun alignNote() = Commands.sequence(
        Commands.runOnce({ noteSimulation?.animateAlign() }),
        rollers.runGoal(Rollers.Goal.ALIGN_FORWARD).until(Beambreak.upperBeambreak::isTripped).afterSimDelay(0.2) { Beambreak.upperBeambreak.setSimTripped() },
    )
        .onlyIf { Beambreak.hasNote }
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        .withTimeout(3.0)
        .withName("Note Align")
}