package org.team9432.resources.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import org.team9432.annotation.Logged

interface ModuleIO {
    @Logged
    open class ModuleIOInputs {
        var drivePositionRotations: Double = 0.0
        var driveVelocityRadPerSecond: Double = 0.0
        var driveAppliedVolts: Double = 0.0
        var driveCurrentAmps: Double = 0.0

        var steerAbsolutePosition: Rotation2d = Rotation2d()
        var steerPosition: Rotation2d = Rotation2d()
        var steerVelocityRadPerSec: Double = 0.0
        var steerAppliedVolts: Double = 0.0
        var steerCurrentAmps: Double = 0.0

        var odometryDrivePositionsRotations: DoubleArray = doubleArrayOf()
        var odometrySteerPositions: Array<Rotation2d> = arrayOf()

        var hardwareConnected: Boolean = false
    }

    /**
     * Updates the inputs
     */
    fun updateInputs(inputs: ModuleIOInputs)

    fun setDriveVoltage(volts: Double) {}

    fun setSteerVoltage(volts: Double) {}

    /**
     * Enable or disable brake mode on the drive motor.
     */
    fun setDriveBrake(enable: Boolean) {}

    /**
     * Enable or disable brake mode on the turn motor.
     */
    fun setSteerBrake(enable: Boolean) {}
}