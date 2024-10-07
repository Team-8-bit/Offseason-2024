package org.team9432.resources.flywheels

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team9432.Robot
import org.team9432.RobotPosition.distanceToSpeaker
import org.team9432.lib.unit.Length
import org.team9432.lib.unit.inMeters
import org.team9432.lib.unit.meters
import kotlin.math.abs

class Flywheels(private val io: FlywheelIO): SubsystemBase() {
    private val inputs = LoggedFlywheelIOInputs()

    private val topShooterMap = InterpolatingDoubleTreeMap()
    private val bottomShooterMap = InterpolatingDoubleTreeMap()

    private var goal = Goal.IDLE
    private var currentTargetSpeeds = ShooterSpeeds(0.0, 0.0)

    enum class Goal(val getSpeeds: () -> ShooterSpeeds) {
        IDLE({ ShooterSpeeds(0.0, 0.0) }),
        VISION_SHOOT({ ShooterSpeeds(0.0, 0.0) }),
        SUBWOOFER({ ShooterSpeeds(2000.0, 5000.0) }),
        FEED_SPEED({ ShooterSpeeds(4000.0, 4000.0) }),
        AMP({ ShooterSpeeds(110.0, 4600.0) });
    }

    init {
        addMapValue(2.25.meters, ShooterSpeeds(upperRPM = 5000.0, lowerRPM = 2000.0))
        addMapValue(2.0.meters, ShooterSpeeds(upperRPM = 5000.0, lowerRPM = 2800.0))
        addMapValue(1.75.meters, ShooterSpeeds(upperRPM = 4000.0, lowerRPM = 3000.0))
        addMapValue(1.5.meters, ShooterSpeeds(upperRPM = 4000.0, lowerRPM = 4000.0))
        addMapValue(1.2.meters, ShooterSpeeds(upperRPM = 4500.0, lowerRPM = 3000.0))

        defaultCommand = runGoal(Goal.IDLE)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Flywheels", inputs)

        if (DriverStation.isDisabled()) {
            goal = Goal.IDLE
        }

        currentTargetSpeeds = if (goal == Goal.VISION_SHOOT) {
            getMapValue(distanceToSpeaker())
        } else {
            goal.getSpeeds()
        }

        val (upperRPM, lowerRPM) = currentTargetSpeeds
        io.runVelocity(upperRPM, lowerRPM)

        Logger.recordOutput("Flywheels/UpperSetpoint", upperRPM)
        Logger.recordOutput("Flywheels/LowerSetpoint", lowerRPM)
        Logger.recordOutput("Flywheels/SpeakerDistance", distanceToSpeaker().inMeters)
        Logger.recordOutput("Flywheels/Goal", goal)
    }

    fun runGoal(newGoal: Goal): Command = startEnd({ this.goal = newGoal }, { goal = Goal.IDLE })

    fun atSpeed() = (
            abs(inputs.lowerVelocityRPM - currentTargetSpeeds.lowerRPM) < 300 &&
                    abs(inputs.upperVelocityRPM - currentTargetSpeeds.upperRPM) < 300 &&
                    !currentTargetSpeeds.isIdle
            ) || Robot.isSimulated // Ignore speed in sim as the flywheels aren't simulated yet

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