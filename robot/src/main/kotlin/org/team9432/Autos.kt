package org.team9432

import choreo.Choreo
import choreo.auto.AutoFactory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.team9432.lib.util.afterSimCondition
import org.team9432.lib.util.afterSimDelay
import org.team9432.resources.drive.Drive
import org.team9432.resources.drive.controllers.ChoreoTrajectoryController
import org.team9432.resources.flywheels.Flywheels
import org.team9432.resources.pivot.Pivot
import org.team9432.resources.rollers.Rollers
import java.util.*

class Autos(
    private val drive: Drive,
    private val pivot: Pivot,
    private val rollers: Rollers,
    private val flywheels: Flywheels,
    private val noteSimulation: NoteSimulation?,
) {
    private val controller = ChoreoTrajectoryController()

    private val factory = AutoFactory(
        RobotPosition::currentPose,
        Choreo.ControlFunction<SwerveSample> { pose, sample -> controller.calculate(pose, sample) },
        drive::acceptTrajectoryInput,
        { Robot.alliance == DriverStation.Alliance.Red },
        drive,
        AutoFactory.ChoreoAutoBindings(),
        Optional.empty()
    )

    fun farsideTriple(): Command {
        val loop = factory.newLoop("FarsideTriple")

        val preload = factory.trajectory("Farside_Triple", loop)

        loop.enabled().whileTrue(flywheels.runGoal(Flywheels.Goal.SHOOT))

        loop.enabled().onTrue(
            Commands.runOnce({ RobotPosition.resetOdometry(preload.initialPose.get()) })
                .andThen(
                    preload.cmd(),
//                    aimAndScore(),
//                    Commands.waitSeconds(2.0),
//                    c5.cmd()
                )
        )

//        c5.active().whileTrue(intake())
//        c5.done().onTrue(aimAndScore().andThen(c4.cmd()))
//
//        c4.active().whileTrue(intake())
//        c4.done().onTrue(aimAndScore().andThen(c3.cmd()))
//
//        c3.active().whileTrue(intake())
//        c3.done().onTrue(aimAndScore())

        loop.enabled().onFalse(Commands.runOnce({drive.clearTrajectoryInput()}))

        return loop.cmd()
    }

    fun aimAndScore() = Commands.parallel(
        Commands.waitSeconds(1.0)
//        drive.aimSpeaker(),
//        rollers.runGoal(Rollers.Goal.SHOOTER_FEED)
    ).withTimeout(0.5)

    // To add to AutoCommands
    fun Drive.aimSpeaker() = Commands.startEnd(
        { setAutoAimGoal { RobotPosition.getStandardAimingParameters().drivetrainAngle } },
        { clearAutoAimGoal() }
    )

    fun intake() = Commands.parallel(
//        pivot.runGoal(Pivot.Goal.INTAKE),
//        Commands.waitUntil(pivot::atGoal).andThen(
//            rollers.runGoal(Rollers.Goal.INTAKE)
//                .until(Beambreak::hasNote).afterSimCondition({ noteSimulation!!.hasNote }, { Beambreak.lowerBeambreak.setSimTripped() })
//                .andThen(alignNote())
//        )
    ).withName("AutoIntake")

    private fun alignNote() = Commands.sequence(
        Commands.runOnce({ noteSimulation?.animateAlign() }),
        rollers.runGoal(Rollers.Goal.ALIGN_FORWARD).until(Beambreak.upperBeambreak::isTripped).afterSimDelay(0.2) { Beambreak.upperBeambreak.setSimTripped() },
    )
        .onlyIf { Beambreak.hasNote }
        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        .withTimeout(3.0)
        .withName("Note Align")
}