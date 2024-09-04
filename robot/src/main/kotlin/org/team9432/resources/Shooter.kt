package org.team9432.resources

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.littletonrobotics.junction.Logger
import org.team9432.PositionConstants
import org.team9432.Robot
import org.team9432.Vision
import org.team9432.lib.KSysIdConfig
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.SysIdUtil
import org.team9432.lib.resource.Resource
import org.team9432.lib.unit.*
import org.team9432.lib.util.angleTo
import org.team9432.lib.util.distanceTo
import org.team9432.lib.util.velocityLessThan
import org.team9432.resources.swerve.Swerve
import kotlin.math.abs

object Shooter: Resource("Shooter") {
    private val topMotor = CANSparkFlex(14, CANSparkLowLevel.MotorType.kBrushless)
    private val bottomMotor = CANSparkFlex(13, CANSparkLowLevel.MotorType.kBrushless)

    private var state = State.IDLE

    private val topShooterMap = InterpolatingDoubleTreeMap()
    private val bottomShooterMap = InterpolatingDoubleTreeMap()

    private val ff = SimpleMotorFeedforward(0.0, 0.0021, 0.0)

    enum class State(val getVoltages: () -> ShooterSpeeds) {
        IDLE({ ShooterSpeeds(0.0, 0.0) }),
        VISION_SHOOT({ getMapValue(distanceToSpeaker() + SmartDashboard.getNumber("Shooter/DistanceOffset", 0.0).inches) }),
        SUBWOOFER({ ShooterSpeeds(2000.0, 5000.0) }),
        FEED_SPEED({ ShooterSpeeds(4000.0, 4000.0) }),
        DASHBOARD_SPEEDS({ ShooterSpeeds(SmartDashboard.getNumber("Shooter/TopTargetSpeed", 0.0), SmartDashboard.getNumber("Shooter/BottomTargetSpeed", 0.0)) }),
        AMP({ ShooterSpeeds(100.0, 4500.0) });
    }

    init {
        RobotPeriodicManager.startPeriodic { trackState(); log() }

        SmartDashboard.putNumber("Shooter/TopTargetSpeed", 100.0)
        SmartDashboard.putNumber("Shooter/BottomTargetSpeed", 4500.0)
        SmartDashboard.putNumber("Shooter/Tuning/ffV", 0.0)
        SmartDashboard.putNumber("Shooter/DistanceOffset", 0.0)

        topMotor.inverted = false
        topMotor.idleMode = CANSparkBase.IdleMode.kBrake
        topMotor.enableVoltageCompensation(10.0)
        topMotor.openLoopRampRate = 0.0
        topMotor.setSmartCurrentLimit(25)

        bottomMotor.inverted = true
        bottomMotor.idleMode = CANSparkBase.IdleMode.kBrake
        bottomMotor.enableVoltageCompensation(10.0)
        bottomMotor.openLoopRampRate = 0.0
        bottomMotor.setSmartCurrentLimit(25)

        addMapValue(2.0.meters, ShooterSpeeds(top = 5000.0, bottom = 2750.0))
        addMapValue(1.75.meters, ShooterSpeeds(top = 5000.0, bottom = 3000.0))
        addMapValue(1.5.meters, ShooterSpeeds(top = 2500.0, bottom = 4000.0))
        addMapValue(1.25.meters, ShooterSpeeds(top = 4000.0, bottom = 5000.0))
    }

    private var currentTargetSpeeds = ShooterSpeeds(0.0, 0.0)

    private fun trackState() {
        currentTargetSpeeds = state.getVoltages()
        val (topRPM, bottomRPM) = currentTargetSpeeds

        Logger.recordOutput("Shooter/TopAtSpeed", (topMotor.encoder.velocity - topRPM) < 300)
        Logger.recordOutput("Shooter/BottomAtSpeed", (bottomMotor.encoder.velocity - bottomRPM) < 300)

        topMotor.setVoltage(ff.calculate(topRPM))
        bottomMotor.setVoltage(ff.calculate(bottomRPM))
    }

    fun isReadyToShootSpeaker(): Boolean {
        return if (Robot.isAutonomous) {
            (flywheelsAtSpeed() || Robot.isSimulated)
        } else {
            if (Vision.isEnabled) {
                distanceToSpeaker() < 2.3.meters &&
                        Swerve.getRobotRelativeSpeeds().velocityLessThan(metersPerSecond = 0.5, rotationsPerSecond = 0.25) &&
                        isAimedAtSpeaker() &&
                        (flywheelsAtSpeed() || Robot.isSimulated) // Ignore speed in sim as the flywheels aren't simulated yet
            } else {
                Swerve.getRobotRelativeSpeeds().velocityLessThan(metersPerSecond = 0.5, rotationsPerSecond = 0.25) &&
                        (flywheelsAtSpeed() || Robot.isSimulated)
            }
        }
    }

    private fun log() {
        Logger.recordOutput("Shooter/SpeakerDistance", distanceToSpeaker().inMeters)
        Logger.recordOutput("Shooter/isReadyToShoot", isReadyToShootSpeaker())
        Logger.recordOutput("Shooter/AimingError", getAimingErrorDegrees())
    }

    private fun isAimedAtSpeaker(): Boolean {
        return if (distanceToSpeaker().inMeters > 1.5) {
            getAimingErrorDegrees() < 10
        } else if (distanceToSpeaker().inMeters > 1.25) {
            getAimingErrorDegrees() < 15
        } else {
            getAimingErrorDegrees() < 25
        }
    }

    private fun getAimingErrorDegrees() = abs((Swerve.getRobotTranslation().angleTo(PositionConstants.speakerAimPose).asRotation2d - Swerve.getRobotPose().rotation).degrees)

    fun flywheelsAtSpeed(rpmTolerance: Int = 300): Boolean {
        val (topTarget, bottomTarget) = currentTargetSpeeds
        return abs(bottomMotor.encoder.velocity - bottomTarget) < rpmTolerance && abs(topMotor.encoder.velocity - topTarget) < rpmTolerance && !currentTargetSpeeds.isIdle
    }

    val isIdle get() = state == State.IDLE
    val isShootingSpeaker get() = state == State.VISION_SHOOT || state == State.SUBWOOFER
    val isShootingAmp get() = state == State.AMP

    fun setState(state: State) {
        this.state = state
        trackState()
        Logger.recordOutput("Shooter/State", Shooter.state)
    }

    /** Return the distance from the robot to the speaker. */
    fun distanceToSpeaker() = Swerve.getRobotTranslation().distanceTo(PositionConstants.speakerAimPose)

    data class ShooterSpeeds(val top: Double, val bottom: Double) {
        val isIdle = top == 0.0 && bottom == 0.0
    }

    private fun addMapValue(distance: Length, speeds: ShooterSpeeds) {
        topShooterMap.put(distance.inMeters, speeds.top)
        bottomShooterMap.put(distance.inMeters, speeds.bottom)
    }

    private fun getMapValue(distance: Length): ShooterSpeeds {
        val topSpeed = topShooterMap.get(distance.inMeters)
        val bottomSpeed = bottomShooterMap.get(distance.inMeters)
        return ShooterSpeeds(topSpeed, bottomSpeed)
    }

    fun getSysId() = SysIdUtil.getSysIdTests(
        config = KSysIdConfig(
            rampRate = 0.5,
            stepVoltage = 12.0,
            timeout = 10.0,
        ),
        setMotors = { volts -> topMotor.setVoltage(volts) }
    )
}