package org.team9432.resources

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkFlex
import com.revrobotics.CANSparkLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.team9432.lib.KSysIdConfig
import org.team9432.lib.KSysIdMechanism
import org.team9432.lib.resource.Action
import org.team9432.lib.resource.Resource
import org.team9432.lib.resource.toAction
import org.team9432.lib.robot.CoroutineRobot
import org.team9432.lib.unit.Length
import org.team9432.lib.unit.inMeters
import org.team9432.lib.unit.meters
import org.team9432.lib.util.enumValue
import org.team9432.lib.util.set
import org.team9432.resources.swerve.Swerve

object Shooter: Resource("Shooter") {
    private val topMotor = CANSparkFlex(14, CANSparkLowLevel.MotorType.kBrushless)
    private val bottomMotor = CANSparkFlex(13, CANSparkLowLevel.MotorType.kBrushless)

    private var state by table.enumValue("State", State.IDLE)

    private val topTargetSpeedEntry = table.getEntry("TopTargetSpeed")
    private val bottomTargetSpeedEntry = table.getEntry("BottomTargetSpeed")
    private val pidP = table.getEntry("pidP")
    private val pidI = table.getEntry("pidI")
    private val pidD = table.getEntry("pidD")
    private val ffV = table.getEntry("ffV")
    private val ffA = table.getEntry("ffA")

    private val topShooterMap = InterpolatingDoubleTreeMap()
    private val bottomShooterMap = InterpolatingDoubleTreeMap()

    private val pid = PIDController(0.0, 0.0, 0.0)
    private var ff = SimpleMotorFeedforward(0.0, 0.0, 0.0)

    enum class State(val getVoltages: () -> ShooterSpeeds) {
        IDLE({ ShooterSpeeds(0.0, 0.0) }),
        SHOOT({ ShooterSpeeds(8.0, 8.0) }),
        VISION_SHOOT({ getMapValue(Swerve.distanceToSpeaker()) }),
        SUBWOOFER({ ShooterSpeeds(6.0, 10.0) }),
        DASHBOARD_SPEEDS({ ShooterSpeeds(topTargetSpeedEntry.getDouble(0.0), bottomTargetSpeedEntry.getDouble(0.0)) }),
        AMP({ ShooterSpeeds(1.0, 5.0) });
    }

    init {
        CoroutineRobot.startPeriodic { periodic() }

        topTargetSpeedEntry.setValue(1000.0)
        bottomTargetSpeedEntry.setValue(0.0)

        pidP.setValue(0.0039231)
        pidI.setValue(0.0)
        pidD.setValue(0.0)
        ffV.setValue(0.0105)
        ffA.setValue(0.0038234)

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

    fun periodic() {
        val newVoltages = state.getVoltages()
        val (topVoltage, bottomVoltage) = newVoltages

//        val actualSetpoint = Units.rotationsPerMinuteToRadiansPerSecond(topVoltage)
//
//        topMotor.setVoltage((ff.calculate(actualSetpoint) * 2) + pid.calculate(Units.rotationsPerMinuteToRadiansPerSecond(topMotor.encoder.velocity), actualSetpoint))
//
//        pid.p = pidP.getDouble(0.0)
//        pid.i = pidI.getDouble(0.0)
//        pid.d = pidD.getDouble(0.0)
//
//        ff = SimpleMotorFeedforward(
//            0.0,
//            ffV.getDouble(0.0),
//            ffA.getDouble(0.0)
//        )

        setVoltage(topVoltage, bottomVoltage)

        table.set("positionRad", Units.rotationsToRadians(topMotor.encoder.position))
        table.set("appliedVolts", topMotor.appliedOutput * topMotor.busVoltage)
        table.set("currentAmps", topMotor.outputCurrent)
        table.set("velocityRadPerSec", Units.rotationsPerMinuteToRadiansPerSecond(topMotor.encoder.velocity))
    }

    fun set(state: State) {
        this.state = state
    }

    private fun setVoltage(topVoltage: Double, bottomVoltage: Double) {
        topMotor.setVoltage(topVoltage)
        bottomMotor.setVoltage(bottomVoltage)
    }

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

    fun getSysId(): SysIdTestContainer {
        val routine = SysIdRoutine(
            KSysIdConfig(
                rampRate = 0.5,
                stepVoltage = 12.0,
                timeout = 10.0,
            ) { table["SysIdState"] = it },
            KSysIdMechanism { volts -> topMotor.setVoltage(volts) }
        )

        return SysIdTestContainer(
            routine.quasistatic(SysIdRoutine.Direction.kForward).toAction(),
            routine.quasistatic(SysIdRoutine.Direction.kReverse).toAction(),
            routine.dynamic(SysIdRoutine.Direction.kForward).toAction(),
            routine.dynamic(SysIdRoutine.Direction.kReverse).toAction(),
        )
    }

    data class SysIdTestContainer(
        val quasistaticForward: Action,
        val quasistaticReverse: Action,
        val dynamicForward: Action,
        val dynamicReverse: Action,
    )
}