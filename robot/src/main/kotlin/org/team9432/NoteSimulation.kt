package org.team9432

import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.FunctionalCommand
import org.littletonrobotics.junction.Logger
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.simulation.competitionfield.objects.Crescendo2024FieldObjects
import org.team9432.lib.simulation.competitionfield.objects.GamePieceOnFlyDisplay
import org.team9432.lib.simulation.competitionfield.simulations.IntakeSimulation
import org.team9432.lib.unit.inMeters
import org.team9432.lib.util.allianceSwitch
import org.team9432.lib.util.distanceTo


class NoteSimulation(private val robotPoseSupplier: () -> Pose2d, private val addGamePieceOnFlyDisplay: (GamePieceOnFlyDisplay) -> Unit, private val intakeSim: IntakeSimulation) {
    private val loadedTransform = Transform3d(Units.inchesToMeters(-5.7), 0.0, Units.inchesToMeters(12.75), Rotation3d(0.0, Math.toRadians(-50.0), 0.0))
    private val aligningTransform = Transform3d(Units.inchesToMeters(-7.6), 0.0, Units.inchesToMeters(10.46), Rotation3d(0.0, Math.toRadians(-50.0), 0.0))

    private var robotNoteTransform: Transform3d? = null
    private var shotNotePose: Pose3d? = null

    init {
        RobotPeriodicManager.startPeriodic { periodic() }
    }

    /** Renders the current position of notes on the field. */
    private fun periodic() {
        val notes = mutableSetOf<Pose3d>()

        // Robot relative note (i.e. one stored in the loader)
        robotNoteTransform?.let { notes.add(Pose3d(robotPoseSupplier.invoke()).transformBy(it)) }

        // Field-relative note (i.e. one moving from the robot to the speaker)
        shotNotePose?.let { notes.add(it) }

        Logger.recordOutput("NoteSimulation/Notes", *notes.toTypedArray())
        Logger.recordOutput("NoteSimulation/IntakeSimPieces", intakeSim.gamePieceCount)
    }

    fun addPreload() {
        intakeSim.gamePieceCount = 1
        robotNoteTransform = loadedTransform
        Beambreak.lowerBeambreak.setSimTripped()
        Beambreak.upperBeambreak.setSimTripped()
    }

    val hasNote get() = intakeSim.gamePieceCount == 1

    /******** Animations ********/

    private val blueSpeaker = Translation3d(0.225, 5.55, 2.1)
    private val redSpeaker = Translation3d(16.317, 5.55, 2.1)

    private val SHOT_SPEED_MPS = 6.0

    fun animateShoot() {
        val startPose = Pose3d(robotPoseSupplier.invoke()).transformBy(loadedTransform).translation
        allianceSwitch(blue = blueSpeaker, red = redSpeaker)

        val duration = robotPoseSupplier.invoke().translation.distanceTo(PositionConstants.speakerAimPose).inMeters / SHOT_SPEED_MPS

        val display = Crescendo2024FieldObjects.NoteOnFly(startPose, duration)
        addGamePieceOnFlyDisplay.invoke(display)

        clearNote()
    }

    fun clearNote() {
        robotNoteTransform = null
        intakeSim.gamePieceCount = 0
    }

    fun animateAlign() {
        animateRobotRelative(loadedTransform, aligningTransform, 0.15)
        animateRobotRelative(aligningTransform, loadedTransform, 0.2)
    }

    private fun animateRobotRelative(start: Transform3d, end: Transform3d, durationSeconds: Double) {
        genericAnimation(durationSeconds) { percentageComplete ->
            val currentPose = start.interpolate(end, percentageComplete)
            robotNoteTransform = Transform3d(currentPose.translation, currentPose.rotation)
        }
    }

    private fun genericAnimation(durationSeconds: Double, onEnd: (() -> Unit)? = null, onLoop: (Double) -> Unit) {
        val timer = Timer()
        FunctionalCommand(
            { timer.restart() },
            { onLoop.invoke(timer.get() / durationSeconds) },
            { timer.stop(); onEnd?.invoke() },
            { timer.hasElapsed(durationSeconds) }
        ).schedule()
    }

    private fun Transform3d.interpolate(endValue: Transform3d, t: Double): Transform3d {
        val start = Pose3d(this.translation, this.rotation)
        val end = Pose3d(endValue.translation, endValue.rotation)
        val result = start.interpolate(end, t)
        return Transform3d(result.translation, result.rotation)
    }
}