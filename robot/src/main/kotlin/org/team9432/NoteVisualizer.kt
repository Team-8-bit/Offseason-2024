package org.team9432

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.team9432.lib.coroutines.RobotScope
import org.team9432.lib.doglog.Logger
import org.team9432.lib.unit.inMeters
import org.team9432.lib.util.allianceSwitch
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.time.Duration.Companion.milliseconds


object NoteVisualizer {
    private val loadedTransform = Transform3d(Units.inchesToMeters(-5.7), 0.0, Units.inchesToMeters(12.75), Rotation3d(0.0, Math.toRadians(-50.0), 0.0))
    private val aligningTransform = Transform3d(Units.inchesToMeters(-7.6), 0.0, Units.inchesToMeters(10.46), Rotation3d(0.0, Math.toRadians(-50.0), 0.0))

    private var robotNoteTransform: Transform3d? = null
    private var shotNotePose: Pose3d? = null

    init {
        RobotScope.launch {
            while (true) {
                val notes = mutableSetOf<Pose3d>()

                robotNoteTransform?.let { notes.add(Pose3d(Swerve.getRobotPose()).transformBy(it)) }
                shotNotePose?.let { notes.add(it) }

                Logger.log("NoteVisualizer", notes.toTypedArray())

                delay(5.milliseconds)
            }
        }
    }

    private val blueSpeaker = Translation3d(0.225, 5.55, 2.1)
    private val redSpeaker = Translation3d(16.317, 5.55, 2.1)

    private const val SHOT_SPEED_MPS = 6.0

    fun shoot() {
        val startPose = Pose3d(Swerve.getRobotPose()).transformBy(loadedTransform)
        val endPose = Pose3d(allianceSwitch(blue = blueSpeaker, red = redSpeaker), startPose.rotation)

        val duration = Shooter.distanceToSpeaker().inMeters / SHOT_SPEED_MPS

        robotNoteTransform = null

        animateFieldRelative(startPose, endPose, duration)
    }

    fun align() {
        animateRobotRelative(loadedTransform, aligningTransform, 0.15)
        animateRobotRelative(aligningTransform, loadedTransform, 0.2)
    }

    private fun animateFieldRelative(start: Pose3d, end: Pose3d, durationSeconds: Double) {
        genericAnimation(durationSeconds, onEnd = { shotNotePose = null }) { percentageComplete ->
            shotNotePose = start.interpolate(end, percentageComplete)
        }
    }

    private fun animateRobotRelative(start: Transform3d, end: Transform3d, durationSeconds: Double) {
        genericAnimation(durationSeconds) { percentageComplete ->
            val currentPose = start.interpolate(end, percentageComplete)
            robotNoteTransform = Transform3d(currentPose.translation, currentPose.rotation)
        }
    }

    private fun genericAnimation(durationSeconds: Double, onEnd: (() -> Unit)? = null, onLoop: (Double) -> Unit) {
        val timer = Timer()
        RobotScope.launch {
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