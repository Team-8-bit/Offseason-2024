package org.team9432.resources.swerve.mapleswerve

import org.littletonrobotics.junction.Logger

object MapleTimeUtils {
    fun delay(seconds: Double) {
        try {
            // Convert seconds to total milliseconds
            val totalMillis = (seconds * 1000).toLong()
            // Calculate the remaining nanoseconds
            val nanoPart = ((seconds * 1000 - totalMillis) * 1000000).toInt()

            // Pause the thread for the specified duration in milliseconds and nanoseconds
            Thread.sleep(totalMillis, nanoPart)
        } catch (e: InterruptedException) {
            // Restore the interrupted status
            Thread.currentThread().interrupt()

            // Optionally, handle the interruption, e.g. logging or throwing a runtime exception
            System.err.println("The sleep was interrupted")
        }
    }

    val logTimeSeconds: Double
        get() = Logger.getTimestamp() / 1000000.0

    val realTimeSeconds: Double
        get() = Logger.getRealTimestamp() / 1000000.0
}