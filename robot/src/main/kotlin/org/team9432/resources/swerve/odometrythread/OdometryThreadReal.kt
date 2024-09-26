package org.team9432.resources.swerve.odometrythread

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import org.littletonrobotics.junction.Logger
import org.team9432.resources.swerve.DrivetrainConstants.ODOMETRY_CACHE_CAPACITY
import org.team9432.resources.swerve.DrivetrainConstants.ODOMETRY_FREQUENCY
import org.team9432.resources.swerve.Swerve
import org.team9432.resources.swerve.odometrythread.OdometryThread.OdometryThreadInputs
import java.util.*
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock


object OdometryThreadReal: Thread(), OdometryThread {
    private val timestampsQueue: Queue<Double> = ArrayBlockingQueue(ODOMETRY_CACHE_CAPACITY)
    private val signalsLock: Lock = ReentrantLock()

    private val queues = mutableListOf<Queue<Double>>()

    private var signals: Array<BaseStatusSignal> = emptyArray()

    init {
        name = "OdometryThread"
        isDaemon = true
    }

    @Synchronized override fun start() {
        super<Thread>.start()
    }

    /** Registers a signal to run in the odometry thread and returns a queue that is filled with the received values. */
    fun registerSignal(signal: StatusSignal<Double>): Queue<Double> {
        val queue = ArrayBlockingQueue<Double>(20)
        signalsLock.lock()
        Swerve.odometryLock.lock()
        try {
            signals += signal
            queues.add(queue)
        } finally {
            signalsLock.unlock()
            Swerve.odometryLock.unlock()
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
            Swerve.odometryLock.lock()
            try {
                var timestamp = Logger.getRealTimestamp() / 1e6
//                var totalLatency = 0.0
//
//                for (signal in signals) {
//                    totalLatency += signal.timestamp.latency
//                }
//
//                if (signals.isNotEmpty()) {
//                    // Subtract average latency from the timestamp
//                    timestamp -= totalLatency / signals.size
//                }

                // Add each new value to it's respective queue
                for (i in signals.indices) {
                    queues[i].offer(signals[i].valueAsDouble)
                }
                // Add the timestamp to the queue
                timestampsQueue.offer(timestamp)
            } finally {
                Swerve.odometryLock.unlock()
            }
        }
    }

    override fun updateInputs(inputs: OdometryThreadInputs) {
        inputs.measurementTimestamps = DoubleArray(timestampsQueue.size)
        var i = 0
        while (timestampsQueue.isNotEmpty()) {
            inputs.measurementTimestamps[i] = timestampsQueue.poll()
            i++
        }
    }
}