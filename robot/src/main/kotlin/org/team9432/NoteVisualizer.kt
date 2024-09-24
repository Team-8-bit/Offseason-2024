package org.team9432

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.littletonrobotics.junction.Logger
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.simulation.competitionfield.objects.Crescendo2024FieldObjects
import org.team9432.lib.unit.inMeters
import org.team9432.lib.util.allianceSwitch
import org.team9432.lib.util.distanceTo
import org.team9432.lib.util.whenSimulated
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration.Companion.milliseconds


object NoteVisualizer {
    private val loadedTransform = Transform3d(Units.inchesToMeters(-5.7), 0.0, Units.inchesToMeters(12.75), Rotation3d(0.0, Math.toRadians(-50.0), 0.0))
    private val aligningTransform = Transform3d(Units.inchesToMeters(-7.6), 0.0, Units.inchesToMeters(10.46), Rotation3d(0.0, Math.toRadians(-50.0), 0.0))

    private var robotNoteTransform: Transform3d? = null
    private var shotNotePose: Pose3d? = null

    init {
        whenSimulated { // We don't want this running on the actual robot
            RobotPeriodicManager.startPeriodic { render() }
        }
    }

    /** Renders the current position of notes on the field. */
    private fun render() {
        val notes = mutableSetOf<Pose3d>()

        // Robot relative note (i.e. one stored in the loader)
        robotNoteTransform?.let { notes.add(Pose3d(Swerve.robotSimulation.getActualRobotPose()).transformBy(it)) }

        // Field-relative note (i.e. one moving from the robot to the speaker)
        shotNotePose?.let { notes.add(it) }

        Logger.recordOutput("NoteVisualizer", *notes.toTypedArray())
    }

    /******** Animations ********/

    private val blueSpeaker = Translation3d(0.225, 5.55, 2.1)
    private val redSpeaker = Translation3d(16.317, 5.55, 2.1)

    private const val SHOT_SPEED_MPS = 6.0

    fun animateShoot() {
        if (!Robot.isSimulated) return

        val startPose = Pose3d(Swerve.robotSimulation.getActualRobotPose()).transformBy(loadedTransform).translation
        allianceSwitch(blue = blueSpeaker, red = redSpeaker)

        val duration = Swerve.robotSimulation.getActualRobotPose().translation.distanceTo(PositionConstants.speakerAimPose).inMeters / SHOT_SPEED_MPS

        val display = Crescendo2024FieldObjects.NoteOnFly(startPose, duration)
        Swerve.robotSimulation.competitionFieldVisualizer.addGamePieceOnFly(display)

        robotNoteTransform = null
    }

    fun clearNote() {
        if (!Robot.isSimulated) return
        robotNoteTransform = null
    }

    fun animateAlign() {
        if (!Robot.isSimulated) return
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
        Robot.coroutineScope.launch {
            timer.restart()

            while (!timer.hasElapsed(durationSeconds)) {
                onLoop.invoke(timer.get() / durationSeconds)
                delay(5.milliseconds)
            }

            timer.stop()
            onEnd?.invoke()
        }
    }

    private fun Transform3d.interpolate(endValue: Transform3d, t: Double): Transform3d {
        val start = Pose3d(this.translation, this.rotation)
        val end = Pose3d(endValue.translation, endValue.rotation)
        val result = start.interpolate(end, t)
        return Transform3d(result.translation, result.rotation)
    }
}