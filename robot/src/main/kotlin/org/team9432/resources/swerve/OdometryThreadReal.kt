// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/
package org.team9432.resources.swerve

import com.ctre.phoenix6.BaseStatusSignal
import org.team9432.resources.swerve.DriveTrainConstants.ODOMETRY_CACHE_CAPACITY
import org.team9432.resources.swerve.DriveTrainConstants.ODOMETRY_WAIT_TIMEOUT_SECONDS
import org.team9432.resources.swerve.OdometryThread.OdometryDoubleInput
import org.team9432.resources.swerve.OdometryThread.OdometryThreadInputs
import org.team9432.resources.swerve.mapleswerve.MapleTimeUtils
import java.util.*
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock

class OdometryThreadReal(
    private val odometryDoubleInputs: Array<OdometryDoubleInput>,
    private val statusSignals: Array<BaseStatusSignal>,
): Thread(), OdometryThread {
    private val timeStampsQueue: Queue<Double> = ArrayBlockingQueue(ODOMETRY_CACHE_CAPACITY)
    private val lock: Lock = ReentrantLock()

    init {
        name = "OdometryThread"
        isDaemon = true
    }

    @Synchronized override fun start() {
        if (odometryDoubleInputs.isNotEmpty()) super<Thread>.start()
    }


    override fun run() {
        while (true) odometryPeriodic()
    }

    private fun odometryPeriodic() {
        refreshSignalsAndBlockThread()

        lock.lock()
        timeStampsQueue.offer(estimateAverageTimeStamps())
        for (odometryDoubleInput in odometryDoubleInputs) odometryDoubleInput.cacheInputToQueue()
        lock.unlock()
    }

    private fun refreshSignalsAndBlockThread() {
        BaseStatusSignal.waitForAll(ODOMETRY_WAIT_TIMEOUT_SECONDS, *statusSignals)
    }

    private fun estimateAverageTimeStamps(): Double {
        val currentTime: Double = MapleTimeUtils.realTimeSeconds
        var totalLatency = 0.0
        for (signal in statusSignals) totalLatency += signal.timestamp.latency

        if (statusSignals.isEmpty()) return currentTime
        return currentTime - totalLatency / statusSignals.size
    }


    override fun updateInputs(inputs: OdometryThreadInputs) {
        inputs.measurementTimeStamps = DoubleArray(timeStampsQueue.size)
        var i = 0
        while (i < inputs.measurementTimeStamps.size && !timeStampsQueue.isEmpty()) {
            inputs.measurementTimeStamps[i] = timeStampsQueue.poll()
            i++
        }
    }

    override fun lockOdometry() {
        lock.lock()
    }

    override fun unlockOdometry() {
        lock.unlock()
    }
}