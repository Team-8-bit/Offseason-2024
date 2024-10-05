package org.team9432

import edu.wpi.first.math.geometry.Pose2d
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.simulation.competitionfield.simulations.CompetitionFieldSimulation
import org.team9432.lib.simulation.competitionfield.simulations.Crescendo2024FieldSimulation
import org.team9432.lib.simulation.competitionfield.simulations.HolonomicChassisSimulation

class RobotSim(swerveSim: HolonomicChassisSimulation) {
    val fieldSimulation: CompetitionFieldSimulation = Crescendo2024FieldSimulation(swerveSim)
    val competitionFieldVisualizer = fieldSimulation.competitionField
    init {
        fieldSimulation.placeGamePiecesOnField()

        fieldSimulation.registerIntake(IntakeSim)

        RobotPeriodicManager.startPeriodic {
            fieldSimulation.updateSimulationWorld()
            competitionFieldVisualizer.updateObjectsToDashboardAndTelemetry()
        }
    }
    fun getActualRobotPose(): Pose2d = fieldSimulation.mainRobot.objectOnFieldPose2d
}