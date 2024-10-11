package org.team9432.resources.drive.module

import edu.wpi.first.math.geometry.Rotation2d
import org.team9432.annotation.Logged

interface ModuleIO {
    @Logged
    open class ModuleIOInputs {
        var drivePositionRotations: Double = 0.0
        var driveVelocityRadPerSecond: Double = 0.0
        var driveAppliedVolts: Double = 0.0
        var driveSupplyCurrentAmps: Double = 0.0
        var driveTorqueCurrentAmps: Double = 0.0

        var steerAbsolutePosition: Rotation2d = Rotation2d()
        var steerPosition: Rotation2d = Rotation2d()
        var steerVelocityRadPerSec: Double = 0.0
        var steerAppliedVolts: Double = 0.0
        var steerSupplyCurrentAmps: Double = 0.0
        var steerTorqueCurrentAmps: Double = 0.0

        var odometryDrivePositionsRotations: DoubleArray = doubleArrayOf()
        var odometrySteerPositions: Array<Rotation2d> = arrayOf()

        var driveConnected: Boolean = false
        var steerConnected: Boolean = false
        var cancoderConnected: Boolean = false
    }

    /**
     * Updates the inputs
     */
    fun updateInputs(inputs: ModuleIOInputs) {}

    fun runDriveVoltage(volts: Double) {}

    fun runSteerVoltage(volts: Double) {}

    fun runDriveVelocitySetpoint(velocityRadPerSec: Double, feedforward: Double) {}

    fun runSteerPosition(angle: Rotation2d) {}

    fun setDrivePID(p: Double, i: Double, d: Double) {}

    fun setSteerPID(p: Double, i: Double, d: Double) {}

    /** Enable or disable brake mode on the drive motor. */
    fun setDriveBrake(enable: Boolean) {}

    /** Enable or disable brake mode on the steer motor. */
    fun setSteerBrake(enable: Boolean) {}

    fun stop() {}
}