// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/
package org.team9432.resources.swerve

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import org.littletonrobotics.junction.Logger
import org.team9432.resources.swerve.DriveTrainConstants.ODOMETRY_CACHE_CAPACITY
import org.team9432.resources.swerve.DriveTrainConstants.ODOMETRY_FREQUENCY
import org.team9432.resources.swerve.OdometryThread.OdometryThreadInputs
import java.util.*
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock


object OdometryThreadReal: Thread(), OdometryThread {
    private val timeStampsQueue: Queue<Double> = ArrayBlockingQueue(ODOMETRY_CACHE_CAPACITY)
    private val signalsLock: Lock = ReentrantLock()

    private val queues = mutableListOf<Queue<Double>>()
    private val timestampQueues = mutableListOf<Queue<Double>>()
    private var signals: Array<BaseStatusSignal> = emptyArray()

    init {
        name = "OdometryThread"
        isDaemon = true
    }

    @Synchronized override fun start() {
        if (timestampQueues.isNotEmpty()) super<Thread>.start()
    }

    fun registerSignal(signal: StatusSignal<Double>): Queue<Double> {
        val queue = ArrayBlockingQueue<Double>(20)
        signalsLock.lock()
        SwerveDrive.odometryLock.lock()
        try {
            signals += signal
            queues.add(queue)
        } finally {
            signalsLock.unlock()
            SwerveDrive.odometryLock.unlock()
        }

        return queue
    }

    override fun run() {
        while (true) {
            // Wait for signal updates
            signalsLock.lock()
            try {
                BaseStatusSignal.waitForAll(2.0 / ODOMETRY_FREQUENCY, *signals)
            } finally {
                signalsLock.unlock()
            }

            // Save data
            SwerveDrive.odometryLock.lock()
            try {
                var timestamp = Logger.getRealTimestamp() / 1e6
                var totalLatency = 0.0

                for (signal in signals) {
                    totalLatency += signal.timestamp.latency
                }

                if (signals.isNotEmpty()) {
                    // Subtract average latency from the timestamp
                    timestamp -= totalLatency / signals.size
                }

                // Add each new value to it's respective queue
                for (i in signals.indices) {
                    queues[i].offer(signals[i].valueAsDouble)
                }
                // Add the timestamp to the queue
                for (i in timestampQueues.indices) {
                    timestampQueues[i].offer(timestamp)
                }
            } finally {
                SwerveDrive.odometryLock.unlock()
            }
        }
    }

    override fun updateInputs(inputs: OdometryThreadInputs) {
        inputs.measurementTimestamps = timeStampsQueue.toDoubleArray()
        timeStampsQueue.clear()
    }
}