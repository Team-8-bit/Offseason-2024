package org.team9432

import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation

object Logging {
    fun start() {
        DataLogManager.start()
        DriverStation.startDataLog(DataLogManager.getLog())
        DataLogManager.logNetworkTables(true)
    }
}