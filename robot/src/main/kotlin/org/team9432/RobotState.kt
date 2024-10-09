package org.team9432

import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.Logger
import org.team9432.lib.RobotPeriodicManager
import org.team9432.lib.util.SwerveSetpointGenerator

object RobotState {
    var visionConnected: Boolean = true
    var automationDisabled: Boolean = false
    var pivotEnabled: Boolean = true
    var shootOnMoveEnabled: Boolean = true

    val swerveLimits = SwerveSetpointGenerator.ModuleLimits(
        maxDriveVelocity = 4.0,
        maxDriveAcceleration = 20.0,
        maxSteeringVelocity = Units.degreesToRadians(1080.0)
    )

    init {
        RobotPeriodicManager.startPeriodic { log() }
    }

    private fun log() {
        Logger.recordOutput("RobotState/VisionConnected", visionConnected)
        Logger.recordOutput("RobotState/AutomationDisabled", automationDisabled)
        Logger.recordOutput("RobotState/PivotEnabled", pivotEnabled)
        Logger.recordOutput("RobotState/ShootOnMoveEnabled", shootOnMoveEnabled)
    }
}