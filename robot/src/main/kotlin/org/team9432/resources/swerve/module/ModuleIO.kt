package org.team9432.resources.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import org.team9432.annotation.Logged

interface ModuleIO {
    @Logged
    open class ModuleIOInputs {
        var driveWheelFinalRevolutions: Double = 0.0
        var driveWheelFinalVelocityRevolutionsPerSec: Double = 0.0
        var driveMotorAppliedVolts: Double = 0.0
        var driveMotorCurrentAmps: Double = 0.0

        var steerFacing: Rotation2d = Rotation2d()
        var steerVelocityRadPerSec: Double = 0.0
        var steerMotorAppliedVolts: Double = 0.0
        var steerMotorCurrentAmps: Double = 0.0

        var odometryDriveWheelRevolutions: DoubleArray = doubleArrayOf()
        var odometrySteerPositions: Array<Rotation2d> = arrayOf()

        var hardwareConnected: Boolean = false
    }

    /**
     * Updates the inputs
     */
    fun updateInputs(inputs: ModuleIOInputs)

    /**
     * Run the drive motor at the specified percent speed.
     * @param speedPercent from -1 to 1, where 1 is the forward direction of the wheel
     */
    fun setDriveVoltage(speedPercent: Double) {}

    /**
     * Run the turn motor at the specified percent power.
     * @param powerPercent from -1 to 1, where 1 is counter-clockwise
     */
    fun setSteerPowerPercent(powerPercent: Double) {}

    /**
     * Enable or disable brake mode on the drive motor.
     */
    fun setDriveBrake(enable: Boolean) {}

    /**
     * Enable or disable brake mode on the turn motor.
     */
    fun setSteerBrake(enable: Boolean) {}
}