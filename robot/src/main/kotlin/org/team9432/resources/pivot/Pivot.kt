package org.team9432.resources.pivot

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.team9432.Robot
import org.team9432.RobotState
import org.team9432.lib.dashboard.LoggedTunableNumber
import org.team9432.lib.unit.inDegrees
import kotlin.math.abs

class Pivot(private val io: PivotIO): SubsystemBase() {
    private val kP = LoggedTunableNumber("Pivot/kP", 15.0)
    private val kI = LoggedTunableNumber("Pivot/kI", 0.0)
    private val kD = LoggedTunableNumber("Pivot/kD", 0.1)
    private val kS = LoggedTunableNumber("Pivot/kS", 0.0)
    private val kG = LoggedTunableNumber("Pivot/kG", 0.4)
    private val kV = LoggedTunableNumber("Pivot/kV", 1.0)
    private val kA = LoggedTunableNumber("Pivot/kA", 0.0)

    private val maxVelocity = LoggedTunableNumber("Pivot/MaxVelocityDegreesPerSec", 600.0)
    private val maxAcceleration = LoggedTunableNumber("Pivot/MaxAccelerationDegreesPerSecPerSec", 800.0)

    enum class Goal(private val angleSupplier: () -> Double) {
        // All angles are in degrees
        IDLE({ 0.0 }),
        INTAKE(LoggedTunableNumber("Pivot/IntakeAngleDegrees", 0.0)),
        SPEAKER_AIM({ RobotState.getStandardAimingParameters().pivotAngle.inDegrees }),
        AMP(LoggedTunableNumber("Pivot/AmpAngleDegrees", 0.0)),
        PODIUM(LoggedTunableNumber("Pivot/PodiumAngleDegrees", 19.0)),
        CUSTOM(LoggedTunableNumber("Pivot/CustomGoal", 0.0));

        val angleRads get() = Units.degreesToRadians(angleSupplier.invoke())
    }

    private var goal = Goal.IDLE
    private var runningCharacterization = false

    private val feedback = PIDController(kP.get(), 0.0, kD.get())
    private var feedforward = ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get())

    private var profile = TrapezoidProfile(TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()))
    private var setpointState = TrapezoidProfile.State()

    private val inputs = LoggedPivotIOInputs()

    init {
        defaultCommand = runGoal(Goal.IDLE)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Pivot", inputs)

        LoggedTunableNumber.ifChanged(hashCode(), { (kP, kI, kD) -> feedback.setPID(kP, kI, kD) }, kP, kI, kD)
        LoggedTunableNumber.ifChanged(hashCode(), { (kS, kG, kV, kA) -> feedforward = ArmFeedforward(kS, kG, kV, kA) }, kS, kG, kV, kA)
        LoggedTunableNumber.ifChanged(
            hashCode(),
            { (maxVel, maxAcc) -> profile = TrapezoidProfile(TrapezoidProfile.Constraints(Units.degreesToRadians(maxVel), Units.degreesToRadians(maxAcc))) },
            maxVelocity,
            maxAcceleration
        )

        val disabled = DriverStation.isDisabled()
        if (disabled) {
            io.runVoltage(0.0)
            feedback.reset()
            setpointState = TrapezoidProfile.State(mechanismPositionRadians, 0.0)
        }

        io.setBrakeMode(!disabled) // Coast when disabled

        if (!runningCharacterization && !disabled) {
            setpointState = profile.calculate(
                Robot.period,
                setpointState,
                TrapezoidProfile.State(
                    MathUtil.clamp(
                        goal.angleRads,
                        Units.degreesToRadians(0.0),
                        Units.degreesToRadians(50.69)
                    ), 0.0
                )
            )

//            if (goal == Goal.CUSTOM) setpointState = TrapezoidProfile.State(positionRadians, 0.0)

            io.runVoltage(
                feedforward.calculate(setpointState.position, setpointState.velocity) +
                        feedback.calculate(mechanismPositionRadians, setpointState.position)
            )

            Logger.recordOutput("Pivot/Mechanism/GoalAngle", *goal.angleRads.pivotAngleToMechanismPose3d())
        }

        Logger.recordOutput("Pivot/Mechanism/SetpointAngle", *setpointState.position.pivotAngleToMechanismPose3d())
        Logger.recordOutput("Pivot/Mechanism/Measured", *mechanismPositionRadians.pivotAngleToMechanismPose3d())
        Logger.recordOutput("Pivot/SetpointAngleDegrees", Units.radiansToDegrees(setpointState.position))
        Logger.recordOutput("Pivot/Goal", goal)
    }

    fun runGoal(newGoal: Goal): Command = startEnd({ this.goal = newGoal }, { goal = Goal.IDLE }).withName("Pivot $goal")

    private val mechanismPositionRadians
        get() = Units.rotationsToRadians(if (inputs.absoluteEncoderConnected) inputs.absolutePositionRotations else inputs.leaderPositionRotations)

    @get:AutoLogOutput(key = "Pivot/AtGoal")
    val atGoal get() = abs(setpointState.position - goal.angleRads) < Units.degreesToRadians(.5)

    fun runCharacterization(volts: Double) {
        runningCharacterization = true
        io.runVoltage(volts)
    }

    fun endCharacterization() {
        runningCharacterization = false
    }

    private fun Double.pivotAngleToMechanismPose3d() = arrayOf(
        Pose3d(
            Units.inchesToMeters(-1.0),
            Units.inchesToMeters(0.0),
            Units.inchesToMeters(15.5),
            Rotation3d(
                0.0,
                this,
                0.0
            )
        )
    )
}