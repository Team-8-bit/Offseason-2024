package org.team9432.resources.swerve.gyro

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import org.team9432.resources.swerve.DriveTrainConstants.ODOMETRY_FREQUENCY
import org.team9432.resources.swerve.odometrythread.OdometryThreadReal
import org.team9432.resources.swerve.TunerConstants
import java.util.*

/** IO implementation for Pigeon2  */
class GyroIOPigeon2: GyroIO {
    private val pigeon = Pigeon2(TunerConstants.kPigeonId, TunerConstants.kCANbusName)
    private val yaw: StatusSignal<Double> = pigeon.yaw
    private var yawPositionQueue: Queue<Double>
    private val yawVelocity: StatusSignal<Double> = pigeon.angularVelocityZWorld

    init {
        pigeon.configurator.apply(Pigeon2Configuration())
        pigeon.configurator.setYaw(0.0)

        yawVelocity.setUpdateFrequency(100.0)
        yaw.setUpdateFrequency(ODOMETRY_FREQUENCY)

        yawPositionQueue = OdometryThreadReal.registerSignal(yaw)

        pigeon.optimizeBusUtilization()
    }

    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.valueAsDouble)
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.valueAsDouble)

        inputs.odometryYawPositions = yawPositionQueue.map { Rotation2d.fromDegrees(it) }.toTypedArray()
        yawPositionQueue.clear()
    }

    override fun setAngle(angle: Rotation2d) {
        pigeon.setYaw(angle.degrees)
    }
}