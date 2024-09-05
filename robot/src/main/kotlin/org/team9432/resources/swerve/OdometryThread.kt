package org.team9432.resources.swerve

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import org.team9432.Robot
import org.team9432.annotation.Logged
import org.team9432.resources.swerve.DriveTrainConstants.ODOMETRY_CACHE_CAPACITY
import org.team9432.resources.swerve.DriveTrainConstants.ODOMETRY_FREQUENCY
import org.team9432.resources.swerve.DriveTrainConstants.ODOMETRY_WAIT_TIMEOUT_SECONDS
import org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils.Simulations.SwerveDriveSimulation
import java.util.*
import java.util.concurrent.ArrayBlockingQueue
import java.util.function.Supplier

interface OdometryThread {
    class OdometryDoubleInput(private val signal: Supplier<Double>) {
        internal val queue: Queue<Double> = ArrayBlockingQueue(ODOMETRY_CACHE_CAPACITY)

        fun cacheInputToQueue() {
            queue.offer(signal.get())
        }
    }

    @Logged
    open class OdometryThreadInputs {
        var measurementTimeStamps: DoubleArray = DoubleArray(0)
    }

    fun updateInputs(inputs: OdometryThreadInputs) {}

    fun start() {}

    fun lockOdometry() {}

    fun unlockOdometry() {}

    companion object {
        fun registerSignalInput(signal: StatusSignal<Double>): Queue<Double> {
            signal.setUpdateFrequency(ODOMETRY_FREQUENCY, ODOMETRY_WAIT_TIMEOUT_SECONDS)
            registeredStatusSignals.add(signal)
            return registerInput(signal.asSupplier())
        }

        fun registerInput(supplier: Supplier<Double>): Queue<Double> {
            val odometryDoubleInput = OdometryDoubleInput(supplier)
            registeredInputs.add(odometryDoubleInput)
            return odometryDoubleInput.queue
        }

        fun createInstance(): OdometryThread {
            return when {
                Robot.isNotSimulated -> OdometryThreadReal(
                    registeredInputs.toTypedArray(),
                    registeredStatusSignals.toTypedArray()
                )

                Robot.isSimulated -> SwerveDriveSimulation.OdometryThreadSim()
                Robot.IS_REPLAY -> object : OdometryThread {}
                else -> TODO()
            }
        }

        val registeredInputs: MutableList<OdometryDoubleInput> = ArrayList()
        val registeredStatusSignals: MutableList<BaseStatusSignal> = ArrayList()
    }
}