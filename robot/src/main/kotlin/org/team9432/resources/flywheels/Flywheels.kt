package org.team9432.resources.flywheels

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team9432.Robot
import org.team9432.RobotPosition
import org.team9432.resources.flywheels.DifferentialFlywheelSpeedMap.ShooterSpeeds
import kotlin.math.abs

class Flywheels(private val io: FlywheelIO): SubsystemBase() {
    private val inputs = LoggedFlywheelIOInputs()

    private var goal = Goal.IDLE
    private var currentTargetSpeeds = ShooterSpeeds(0.0, 0.0)

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

        if (DriverStation.isDisabled()) {
            goal = Goal.IDLE
        }

        currentTargetSpeeds = goal.getSpeeds()

        val (upperRPM, lowerRPM) = currentTargetSpeeds
        io.runVelocity(upperRPM, lowerRPM)

        Logger.recordOutput("Flywheels/UpperSetpoint", upperRPM)
        Logger.recordOutput("Flywheels/LowerSetpoint", lowerRPM)
        Logger.recordOutput("Flywheels/Goal", goal)
    }

    fun runGoal(newGoal: Goal): Command = startEnd({ this.goal = newGoal }, { goal = Goal.IDLE })

    fun atSpeed() = (
            abs(inputs.lowerVelocityRPM - currentTargetSpeeds.lowerRPM) < 300 &&
                    abs(inputs.upperVelocityRPM - currentTargetSpeeds.upperRPM) < 300 &&
                    !currentTargetSpeeds.isIdle
            ) || Robot.isSimulated // Ignore speed in sim as the flywheels aren't simulated yet
}