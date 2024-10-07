package org.team9432

import org.littletonrobotics.junction.Logger
import org.team9432.lib.RobotPeriodicManager

object RobotState {
    var visionConnected: Boolean = true
    var automationDisabled: Boolean = false
    var driverRequestedIntake: Boolean = false

    init {
        RobotPeriodicManager.startPeriodic { log() }
    }

    private fun log() {
        Logger.recordOutput("RobotState/VisionConnected", visionConnected)
        Logger.recordOutput("RobotState/AutomationDisabled", automationDisabled)
        Logger.recordOutput("RobotState/DriverRequestedIntake", driverRequestedIntake)
    }
}