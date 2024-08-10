package org.team9432.resources

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlinx.coroutines.delay
import org.team9432.PositionConstants
import org.team9432.lib.Beambreak
import org.team9432.lib.KSysIdConfig
import org.team9432.lib.SysIdUtil
import org.team9432.lib.coroutines.CoroutineRobot
import org.team9432.lib.coroutines.await
import org.team9432.lib.doglog.Logger
import org.team9432.lib.resource.Resource
import org.team9432.lib.unit.Length
import org.team9432.lib.unit.asRotation2d
import org.team9432.lib.unit.inMeters
import org.team9432.lib.unit.meters
import org.team9432.lib.util.angleTo
import org.team9432.lib.util.distanceTo
import org.team9432.lib.util.velocityLessThan
import org.team9432.resources.swerve.Swerve
import kotlin.math.abs
import kotlin.time.Duration.Companion.seconds

object Shooter: Resource("Shooter") {
    private val topMotor = CANSparkFlex(14, CANSparkLowLevel.MotorType.kBrushless)
    private val bottomMotor = CANSparkFlex(13, CANSparkLowLevel.MotorType.kBrushless)

    private var state = State.IDLE

    private val topShooterMap = InterpolatingDoubleTreeMap()
    private val bottomShooterMap = InterpolatingDoubleTreeMap()

    private val pid = PIDController(0.0, 0.0, 0.0)
    private var ff = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    enum class State(val getVoltages: () -> ShooterSpeeds) {
        IDLE({ ShooterSpeeds(0.0, 0.0) }),
        SHOOT({ ShooterSpeeds(8.0, 8.0) }),
        VISION_SHOOT({ getMapValue(distanceToSpeaker()) }),
        SUBWOOFER({ ShooterSpeeds(6.0, 10.0) }),
        DASHBOARD_SPEEDS({ ShooterSpeeds(SmartDashboard.getNumber("Shooter/TopTargetSpeed", 0.0), SmartDashboard.getNumber("Shooter/BottomTargetSpeed", 0.0)) }),
        AMP({ ShooterSpeeds(1.0, 5.0) });
    }

    init {
        CoroutineRobot.startPeriodic { trackState(); log() }

        SmartDashboard.putNumber("Shooter/TopTargetSpeed", 1000.0)
        SmartDashboard.putNumber("Shooter/BottomTargetSpeed", 0.0)

        SmartDashboard.putNumber("Shooter/Tuning/pidP", 0.0039231)
        SmartDashboard.putNumber("Shooter/Tuning/pidI", 0.0)
        SmartDashboard.putNumber("Shooter/Tuning/pidD", 0.0)
        SmartDashboard.putNumber("Shooter/Tuning/ffV", 0.0105)
        SmartDashboard.putNumber("Shooter/Tuning/ffA", 0.0038234)

        pid.setTolerance(Units.rotationsPerMinuteToRadiansPerSecond(100.0))

        topMotor.inverted = false
        topMotor.idleMode = CANSparkBase.IdleMode.kBrake
        topMotor.enableVoltageCompensation(10.0)
        topMotor.openLoopRampRate = 0.0

        bottomMotor.inverted = true
        bottomMotor.idleMode = CANSparkBase.IdleMode.kBrake
        bottomMotor.enableVoltageCompensation(10.0)
        bottomMotor.openLoopRampRate = 0.0

        addMapValue(2.0.meters, ShooterSpeeds(top = 10.0, bottom = 5.0))
        addMapValue(1.5.meters, ShooterSpeeds(top = 8.0, bottom = 8.0))
        addMapValue(1.0.meters, ShooterSpeeds(top = 6.0, bottom = 10.0))
    }

    private fun trackState() {
        val (topVoltage, bottomVoltage) = state.getVoltages()
        topMotor.setVoltage(topVoltage)
        bottomMotor.setVoltage(bottomVoltage)

//        val actualSetpoint = Units.rotationsPerMinuteToRadiansPerSecond(topVoltage)
//        topMotor.setVoltage((ff.calculate(actualSetpoint) * 2) + pid.calculate(Units.rotationsPerMinuteToRadiansPerSecond(topMotor.encoder.velocity), actualSetpoint))
    }

    fun isReadyToShoot(): Boolean {
        // TODO: Add flywheel speed check
        return distanceToSpeaker() < 2.0.meters &&
                Swerve.getRobotSpeeds().velocityLessThan(metersPerSecond = 1.0, rotationsPerSecond = 0.25) &&
                getAimingErrorDegrees() < 10
    }

    suspend fun awaitReady() {
        delay(1.seconds) // This is just until we get the flywheel pid tuned/working
        await { isReadyToShoot() }
    }

    private fun log() {
        Logger.log("Shooter/TopMotor", topMotor)
        Logger.log("Shooter/BottomMotor", bottomMotor)
        Logger.log("Shooter/State", state)
        Logger.log("Shooter/SpeakerDistance", distanceToSpeaker().inMeters)
        Logger.log("Shooter/isReadyToShoot", isReadyToShoot())
        Logger.log("Shooter/AimingError", getAimingErrorDegrees())
    }

    private fun getAimingErrorDegrees() = abs((Swerve.getRobotTranslation().angleTo(PositionConstants.speakerAimPose).asRotation2d - Swerve.getRobotPose().rotation).degrees)

    fun setState(state: State) {
        this.state = state
    }

    /** Return the distance from the robot to the speaker. */
    private fun distanceToSpeaker() = Swerve.getRobotTranslation().distanceTo(PositionConstants.speakerAimPose)

    data class ShooterSpeeds(val top: Double, val bottom: Double)

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