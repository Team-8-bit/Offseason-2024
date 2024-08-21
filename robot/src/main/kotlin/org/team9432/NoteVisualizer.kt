package org.team9432

import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.CancellableContinuation
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.suspendCancellableCoroutine
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.coroutines.RobotScope
import org.team9432.lib.doglog.Logger
import org.team9432.lib.unit.inMeters
import org.team9432.lib.util.allianceSwitch
import org.team9432.lib.util.whenSimulated
import org.team9432.resources.Intake
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import kotlin.coroutines.resume
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds


object NoteVisualizer {
    private val loadedTransform = Transform3d(Units.inchesToMeters(-5.7), 0.0, Units.inchesToMeters(12.75), Rotation3d(0.0, Math.toRadians(-50.0), 0.0))
    private val aligningTransform = Transform3d(Units.inchesToMeters(-7.6), 0.0, Units.inchesToMeters(10.46), Rotation3d(0.0, Math.toRadians(-50.0), 0.0))

    private var robotNoteTransform: Transform3d? = null
    private var shotNotePose: Pose3d? = null

    private val fieldNotes = FieldConstants.allNotes.toMutableSet()

    private var intakeTranslations = mutableSetOf(
        Transform2d(Units.inchesToMeters(-14.325), Units.inchesToMeters(8.0), Rotation2d()),
        Transform2d(Units.inchesToMeters(-14.325), Units.inchesToMeters(4.0), Rotation2d()),
        Transform2d(Units.inchesToMeters(-14.325), Units.inchesToMeters(0.0), Rotation2d()),
        Transform2d(Units.inchesToMeters(-14.325), Units.inchesToMeters(-4.0), Rotation2d()),
        Transform2d(Units.inchesToMeters(-14.325), Units.inchesToMeters(-8.0), Rotation2d())
    )

    init {
        whenSimulated { // We don't want this running on the actual robot
            RobotScope.launch {
                while (true) {
                    render()
                    delay(5.milliseconds)
                }
            }

            RobotPeriodicManager.startPeriodic { checkCollectedNotes() }
        }
    }

    /** Renders the current position of notes on the field. */
    private fun render() {
        val notes = mutableSetOf<Pose3d>()

        // Robot relative note (i.e. one stored in the loader)
        robotNoteTransform?.let { notes.add(Pose3d(Swerve.getRobotPose()).transformBy(it)) }

        // Field-relative note (i.e. one moving from the robot to the speaker)
        shotNotePose?.let { notes.add(it) }

        // Field notes
        notes.addAll(fieldNotes.map { Pose3d(Pose2d(it.x, it.y, Rotation2d())) })

        Logger.log("NoteVisualizer", notes.toTypedArray())
    }


    /******** Intaking Simulation ********/

    private val NOTE_RADIUS_METERS = Units.inchesToMeters(14.0) / 2

    private fun checkCollectedNotes() {
        val currentIntakePositions = intakeTranslations.map { Swerve.getRobotPose().transformBy(it).translation }

        if (Intake.isIntaking && Swerve.getRobotRelativeSpeeds().vxMetersPerSecond < -0.5) {
            fieldNotes.toList().forEach { note ->
                val isBeingCollected = currentIntakePositions.any { intake ->
                    note.getDistance(intake) < NOTE_RADIUS_METERS
                }

                if (isBeingCollected) {
                    awaitingContinuations.forEach { it.resume(Unit) }
                    awaitingContinuations.clear()

                    fieldNotes.remove(note)
                    RobotScope.launch {
                        delay(30.seconds)
                        fieldNotes.add(note)
                    }
                }
            }
        }
    }

    private var awaitingContinuations = mutableSetOf<CancellableContinuation<Unit>>()

    /** Suspends until the robot picks up a simulated note. */
    suspend fun awaitNotePickup() = suspendCancellableCoroutine { continuation ->
        awaitingContinuations.add(continuation)
    }


    /******** Animations ********/

    private val blueSpeaker = Translation3d(0.225, 5.55, 2.1)
    private val redSpeaker = Translation3d(16.317, 5.55, 2.1)

    private const val SHOT_SPEED_MPS = 6.0

    fun animateShoot() {
        if (!Robot.isSimulated) return
        val startPose = Pose3d(Swerve.getRobotPose()).transformBy(loadedTransform)
        val endPose = Pose3d(allianceSwitch(blue = blueSpeaker, red = redSpeaker), startPose.rotation)

        val duration = Shooter.distanceToSpeaker().inMeters / SHOT_SPEED_MPS

        robotNoteTransform = null

        animateFieldRelative(startPose, endPose, duration)
    }

    fun animateAlign() {
        if (!Robot.isSimulated) return
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