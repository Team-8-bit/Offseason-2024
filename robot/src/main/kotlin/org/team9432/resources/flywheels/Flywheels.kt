package org.team9432.resources.flywheels

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team9432.Robot
import org.team9432.RobotPosition
import org.team9432.lib.LinearProfile
import org.team9432.lib.dashboard.LoggedTunableNumber
import org.team9432.resources.flywheels.DifferentialFlywheelSpeedMap.ShooterSpeeds

class Flywheels(private val io: FlywheelIO): SubsystemBase() {
    private val kP = LoggedTunableNumber("Flywheels/kP", 0.0)
    private val kI = LoggedTunableNumber("Flywheels/kI", 0.0)
    private val kD = LoggedTunableNumber("Flywheels/kD", 0.0)
    private val kS = LoggedTunableNumber("Flywheels/kS", 0.0)
    private val kV = LoggedTunableNumber("Flywheels/kV", 0.0021)
    private val kA = LoggedTunableNumber("Flywheels/kA", 0.0)
    private val maxAcceleration = LoggedTunableNumber("Flywheels/MaxAccelerationRPMPerSec", 0.0)
    private val prepareShootPercentage by LoggedTunableNumber("Flywheels/PrepareShootPercentage", 0.75)

    private val inputs = LoggedFlywheelIOInputs()

    private var goal = Goal.IDLE
    private var currentTargetSpeeds = ShooterSpeeds(0.0, 0.0)

    private val upperProfile = LinearProfile(maxAcceleration.get(), Robot.period)
    private val lowerProfile = LinearProfile(maxAcceleration.get(), Robot.period)
    private val upperFeedback = PIDController(kP.get(), kI.get(), kD.get())
    private val lowerFeedback = PIDController(kP.get(), kI.get(), kD.get())
    private var feedforward = SimpleMotorFeedforward(kS.get(), kV.get(), kA.get())

    var shouldPrepareSpeaker = { false }

    enum class Goal(val getSpeeds: () -> ShooterSpeeds) {
        IDLE({ ShooterSpeeds(0.0, 0.0) }),
        SHOOT({ RobotPosition.getStandardAimingParameters().shooterSpeeds }),
        SUBWOOFER({ ShooterSpeeds(2000.0, 5000.0) }),
        NOTE_ALIGN({ ShooterSpeeds(-200.0, -200.0) }),
        FEED_SPEED({ ShooterSpeeds(4000.0, 4000.0) }),
        AMP({ ShooterSpeeds(110.0, 4600.0) });
    }

    init {
        defaultCommand = runGoal(Goal.IDLE)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Flywheels", inputs)

        LoggedTunableNumber.ifChanged(
            hashCode(),
            { (kP, kI, kD) ->
                upperFeedback.setPID(kP, kI, kD)
                lowerFeedback.setPID(kP, kI, kD)
            },
            kP, kI, kD
        )
        LoggedTunableNumber.ifChanged(hashCode(), { (kS, kV, kA) -> feedforward = SimpleMotorFeedforward(kS, kV, kA) }, kS, kV, kA)
        LoggedTunableNumber.ifChanged(
            hashCode(),
            { (maxAccel) ->
                upperProfile.setMaxAcceleration(maxAccel)
                lowerProfile.setMaxAcceleration(maxAccel)
            },
            maxAcceleration
        )

        if (DriverStation.isDisabled()) {
            goal = Goal.IDLE
        }

        currentTargetSpeeds = goal.getSpeeds()
        var upperGoal = currentTargetSpeeds.upperRPM
        var lowerGoal = currentTargetSpeeds.lowerRPM

        // If we should prepare to shoot
        if (goal == Goal.IDLE && shouldPrepareSpeaker.invoke()) {
            val shootSpeeds = Goal.SHOOT.getSpeeds()
            upperGoal = shootSpeeds.upperRPM * prepareShootPercentage
            lowerGoal = shootSpeeds.lowerRPM * prepareShootPercentage
        }

        upperProfile.setGoal(upperGoal, inputs.upperVelocityRPM)
        lowerProfile.setGoal(lowerGoal, inputs.lowerVelocityRPM)

        val upperSetpoint = upperProfile.calculateSetpoint()
        val lowerSetpoint = lowerProfile.calculateSetpoint()

        io.runVoltage(
            upperVoltage = feedforward.calculate(upperSetpoint) + upperFeedback.calculate(inputs.upperVelocityRPM, upperSetpoint),
            lowerVoltage = feedforward.calculate(upperSetpoint) + lowerFeedback.calculate(inputs.lowerVelocityRPM, lowerSetpoint),
        )

        Logger.recordOutput("Flywheels/UpperSetpointRPM", upperSetpoint)
        Logger.recordOutput("Flywheels/LowerSetpointRPM", lowerSetpoint)
        Logger.recordOutput("Flywheels/UpperGoalRPM", upperGoal)
        Logger.recordOutput("Flywheels/LowerGoalRPM", lowerGoal)
        Logger.recordOutput("Flywheels/AtGoal", atGoal)
        Logger.recordOutput("Flywheels/Goal", goal)
    }

    private fun setGoal(goal: Goal) {
        currentTargetSpeeds = goal.getSpeeds()
        val (upperRPM, lowerRPM) = currentTargetSpeeds
        upperProfile.setGoal(upperRPM, inputs.upperVelocityRPM)
        lowerProfile.setGoal(lowerRPM, inputs.lowerVelocityRPM)
    }

    fun runGoal(newGoal: Goal): Command = startEnd({ setGoal(newGoal) }, { setGoal(Goal.IDLE) })

    val atGoal: Boolean
        get() = goal == Goal.IDLE ||
                (upperProfile.currentSetpoint == currentTargetSpeeds.upperRPM &&
                        lowerProfile.currentSetpoint == currentTargetSpeeds.lowerRPM)
}