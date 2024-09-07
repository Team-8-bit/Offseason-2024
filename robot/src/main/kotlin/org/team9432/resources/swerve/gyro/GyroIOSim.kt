package org.team9432.resources.swerve.gyro

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.Logger
import org.team9432.Robot
import org.team9432.resources.swerve.DriveTrainConstants.AVERAGE_VELOCITY_RAD_PER_SEC_DURING_TEST
import org.team9432.resources.swerve.DriveTrainConstants.GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ
import org.team9432.resources.swerve.DriveTrainConstants.NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD
import org.team9432.resources.swerve.DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD
import org.team9432.resources.swerve.DriveTrainConstants.SKIDDING_AMOUNT_AT_THRESHOLD_RAD
import org.team9432.resources.swerve.mapleswerve.MapleTimeUtils
import org.team9432.resources.swerve.mapleswerve.utils.CustomMaths.MapleCommonMath
import kotlin.math.abs
import kotlin.math.sqrt

class GyroIOSim: GyroIO {
    val gyroPhysicsSimulationResults: GyroPhysicsSimulationResults = GyroPhysicsSimulationResults()
    var previousAngularVelocityRadPerSec: Double = gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec
    var currentGyroDriftAmount: Rotation2d = Rotation2d()

    private var currentUnalteredYawPosition: Rotation2d = Rotation2d()
    private var offsetAngle = Rotation2d()

    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        val angularVelocityChange = abs(gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec - previousAngularVelocityRadPerSec)
        val angularAccelerationMagnitudeRadPerSecSq: Double = angularVelocityChange / Robot.period
        previousAngularVelocityRadPerSec = gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec
        val currentTickDriftStdDevRad: Double = if (angularAccelerationMagnitudeRadPerSecSq > GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ)
            angularAccelerationMagnitudeRadPerSecSq * SKIDDING_AMOUNT_AT_THRESHOLD_RAD / GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ
        else abs(gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec) * GYRO_DRIFT_IN_1_TICK_Std_Dev_RAD / AVERAGE_VELOCITY_RAD_PER_SEC_DURING_TEST
        currentGyroDriftAmount = currentGyroDriftAmount.rotateBy(Rotation2d.fromRadians(MapleCommonMath.generateRandomNormal(0.0, currentTickDriftStdDevRad)))

        inputs.connected = gyroPhysicsSimulationResults.hasReading
        inputs.odometryYawPositions = gyroPhysicsSimulationResults.odometryYawPositions.map { robotAngle -> robotAngle.rotateBy(currentGyroDriftAmount).rotateBy(offsetAngle) }.toTypedArray()
        inputs.yawPosition = inputs.odometryYawPositions.get(inputs.odometryYawPositions.size - 1)
        inputs.yawVelocityRadPerSec = gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec

        currentUnalteredYawPosition = gyroPhysicsSimulationResults.odometryYawPositions.first().rotateBy(currentGyroDriftAmount)

        Logger.recordOutput(
            GYRO_LOG_PATH + "robot true yaw (deg)",
            gyroPhysicsSimulationResults.odometryYawPositions[gyroPhysicsSimulationResults.odometryYawPositions.size - 1].degrees
        )
        Logger.recordOutput(GYRO_LOG_PATH + "robot power for (Sec)", MapleTimeUtils.logTimeSeconds)
        Logger.recordOutput(GYRO_LOG_PATH + "imu total drift (Deg)", currentGyroDriftAmount.degrees)
        Logger.recordOutput(GYRO_LOG_PATH + "gyro reading yaw (Deg)", inputs.yawPosition.getDegrees())
        Logger.recordOutput(GYRO_LOG_PATH + "angular velocity (Deg per Sec)", Math.toDegrees(previousAngularVelocityRadPerSec))
        Logger.recordOutput(GYRO_LOG_PATH + "gyro angular acc (Deg per Sec^2)", Math.toDegrees(angularAccelerationMagnitudeRadPerSecSq))
        Logger.recordOutput(GYRO_LOG_PATH + "new drift in current tick Std Dev (Deg)", Math.toDegrees(currentTickDriftStdDevRad))
    }

    override fun setAngle(angle: Rotation2d) {
        offsetAngle = angle.minus(currentUnalteredYawPosition)
    }

    class GyroPhysicsSimulationResults {
        var robotAngularVelocityRadPerSec: Double = 0.0
        var hasReading: Boolean = false

        val odometryYawPositions: Array<Rotation2d> = Array(SIMULATION_TICKS_IN_1_PERIOD) { Rotation2d() }
    }

    companion object {
        val GYRO_LOG_PATH: String = "MaplePhysicsSimulation/GyroSim/"

        /*
    * we know that in one minute, or n=(60 / Robot.defaultPeriodSeconds) periods
    * the gyro's drift has standard deviation of NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD
    * sqrt(n) * GYRO_DRIFT_IN_1_TICK_Std_Dev_RAD = NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD
     *  */
        val GYRO_DRIFT_IN_1_TICK_Std_Dev_RAD: Double = NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD / sqrt(60.0 / Robot.period)
    }
}