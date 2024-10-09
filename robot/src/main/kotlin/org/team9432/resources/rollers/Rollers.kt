package org.team9432.resources.rollers

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.team9432.resources.rollers.intake.Intake
import org.team9432.resources.rollers.loader.Loader

class Rollers(
    private val intake: Intake,
    private val loader: Loader,
): SubsystemBase() {
    private var goal = Goal.IDLE

    enum class Goal {
        IDLE,
        INTAKE,
        SHOOTER_FEED,
        FLOOR_EJECT,
        ALIGN_REVERSE,
        ALIGN_FORWARD
    }

    init {
        defaultCommand = runGoal(Goal.IDLE)
    }

    override fun periodic() {
        if (DriverStation.isDisabled()) {
            goal = Goal.IDLE
        }

        intake.goal = Intake.Goal.IDLE
        loader.goal = Loader.Goal.IDLE

        when (goal) {
            Goal.IDLE -> {}
            Goal.INTAKE -> {
                intake.goal = Intake.Goal.FLOOR_INTAKE
                loader.goal = Loader.Goal.FLOOR_INTAKE
            }

            Goal.SHOOTER_FEED -> {
                loader.goal = Loader.Goal.SHOOTER_FEED
            }

            Goal.FLOOR_EJECT -> {
                intake.goal = Intake.Goal.FLOOR_EJECT
                loader.goal = Loader.Goal.REVERSE
            }

            Goal.ALIGN_REVERSE -> {
                loader.goal = Loader.Goal.ALIGN_REVERSE
            }

            Goal.ALIGN_FORWARD -> {
                loader.goal = Loader.Goal.ALIGN_FORWARD
            }
        }

        intake.periodic()
        loader.periodic()

        Logger.recordOutput("Rollers/Goal", goal)
    }

    val isIntaking get() = goal == Goal.INTAKE

    fun runGoal(newGoal: Goal): Command = startEnd({ goal = newGoal }, { goal = Goal.IDLE })
}