package org.team9432.resources.shooter

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.littletonrobotics.junction.Logger
import org.team9432.PositionConstants
import org.team9432.Robot
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.resource.Resource
import org.team9432.lib.unit.*
import org.team9432.lib.util.angleTo
import org.team9432.lib.util.distanceTo
import org.team9432.lib.util.simSwitch
import org.team9432.lib.util.velocityLessThan
import org.team9432.resources.swerve.Swerve
import org.team9432.vision.Vision
import kotlin.math.abs

object Shooter: Resource("Shooter") {
    private val io = simSwitch(real = { ShooterIONeo() }, sim = { ShooterIOSim() })
    private val inputs = LoggedShooterIOInputs()

    private val topShooterMap = InterpolatingDoubleTreeMap()
    private val bottomShooterMap = InterpolatingDoubleTreeMap()

    private var state = State.IDLE
    private var currentTargetSpeeds = ShooterSpeeds(0.0, 0.0)

    const val SHOT_TIME_SECONDS = 0.4

    enum class State(val getSpeeds: () -> ShooterSpeeds) {
        IDLE({ ShooterSpeeds(0.0, 0.0) }),
        VISION_SHOOT({ getMapValue(distanceToSpeaker() + SmartDashboard.getNumber("Shooter/DistanceOffset", 0.0).inches) }),
        SUBWOOFER({ ShooterSpeeds(2000.0, 5000.0) }),
        FEED_SPEED({ ShooterSpeeds(4000.0, 4000.0) }),
        DASHBOARD_SPEEDS({ ShooterSpeeds(SmartDashboard.getNumber("Shooter/TopTargetSpeed", 0.0), SmartDashboard.getNumber("Shooter/BottomTargetSpeed", 0.0)) }),
        AMP({ ShooterSpeeds(100.0, 4500.0) });
    }

    init {
        RobotPeriodicManager.startPeriodic { trackState(); akitUpdate() }

        SmartDashboard.putNumber("Shooter/TopTargetSpeed", 100.0)
        SmartDashboard.putNumber("Shooter/BottomTargetSpeed", 4500.0)
        SmartDashboard.putNumber("Shooter/Tuning/ffV", 0.0)
        SmartDashboard.putNumber("Shooter/DistanceOffset", 0.0)

        addMapValue(2.25.meters, ShooterSpeeds(upperRPM = 5000.0, lowerRPM = 2000.0))
        addMapValue(2.0.meters, ShooterSpeeds(upperRPM = 5000.0, lowerRPM = 2800.0))
        addMapValue(1.75.meters, ShooterSpeeds(upperRPM = 4000.0, lowerRPM = 3000.0))
        addMapValue(1.5.meters, ShooterSpeeds(upperRPM = 4000.0, lowerRPM = 4000.0))
        addMapValue(1.2.meters, ShooterSpeeds(upperRPM = 4500.0, lowerRPM = 3000.0))
    }

    private fun trackState() {
        currentTargetSpeeds = state.getSpeeds()
        val (upperRPM, lowerRPM) = currentTargetSpeeds
        io.runVelocity(upperRPM, lowerRPM)
    }

    fun isReadyToShootSpeaker(): Boolean {
        return if (Robot.isAutonomous) {
            (flywheelsAtSpeed() || Robot.isSimulated)
        } else {
            if (Vision.isEnabled) {
                distanceToSpeaker() < 2.3.meters &&
                        distanceToSpeaker() > 1.3.meters &&
                        Swerve.getRobotRelativeSpeeds().velocityLessThan(metersPerSecond = 1.5, rotationsPerSecond = 0.25) &&
                        isAimedAtSpeaker() &&
                        (flywheelsAtSpeed() || Robot.isSimulated) // Ignore speed in sim as the flywheels aren't simulated yet
            } else {
                Swerve.getRobotRelativeSpeeds().velocityLessThan(metersPerSecond = 0.5, rotationsPerSecond = 0.25) &&
                        (flywheelsAtSpeed() || Robot.isSimulated)
            }
        }
    }

    private fun isAimedAtSpeaker(): Boolean {
        return if (distanceToSpeaker().inMeters > 1.5) {
            getAimingErrorDegrees() < 5
        } else if (distanceToSpeaker().inMeters > 1.25) {
            getAimingErrorDegrees() < 10
        } else {
            getAimingErrorDegrees() < 25
        }
    }

    private fun getAimingErrorDegrees(): Double {
        return abs((Swerve.getFutureRobotPose(SHOT_TIME_SECONDS).angleTo(PositionConstants.speakerAimPose).asRotation2d - Swerve.getRobotPose().rotation).degrees)
    }

    fun flywheelsAtSpeed(rpmTolerance: Int = 300): Boolean {
        val (upperTarget, lowerTarget) = currentTargetSpeeds
        return abs(inputs.lowerVelocityRPM - lowerTarget) < rpmTolerance && abs(inputs.upperVelocityRPM - upperTarget) < rpmTolerance && !currentTargetSpeeds.isIdle
    }

    val isIdle get() = state == State.IDLE
    val isShootingSpeaker get() = state == State.VISION_SHOOT || state == State.SUBWOOFER
    val isShootingAmp get() = state == State.AMP

    fun setState(state: State) {
        Shooter.state = state
        trackState()
        Logger.recordOutput("Shooter/State", Shooter.state)
    }

    override fun akitUpdate() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter", inputs)

        Logger.recordOutput("Shooter/SpeakerDistance", distanceToSpeaker().inMeters)
        Logger.recordOutput("Shooter/isReadyToShoot", isReadyToShootSpeaker())
        Logger.recordOutput("Shooter/AimingError", getAimingErrorDegrees())
    }

    /** Return the distance from the robot to the speaker. */
    fun distanceToSpeaker(): Length {
        return Swerve.getFutureRobotPose(SHOT_TIME_SECONDS).distanceTo(PositionConstants.speakerAimPose)
    }

    data class ShooterSpeeds(val upperRPM: Double, val lowerRPM: Double) {
        val isIdle = upperRPM == 0.0 && lowerRPM == 0.0
    }

    private fun addMapValue(distance: Length, speeds: ShooterSpeeds) {
        topShooterMap.put(distance.inMeters, speeds.upperRPM)
        bottomShooterMap.put(distance.inMeters, speeds.lowerRPM)
    }

    private fun getMapValue(distance: Length): ShooterSpeeds {
        val topSpeed = topShooterMap.get(distance.inMeters)
        val bottomSpeed = bottomShooterMap.get(distance.inMeters)
        return ShooterSpeeds(topSpeed, bottomSpeed)
    }
}