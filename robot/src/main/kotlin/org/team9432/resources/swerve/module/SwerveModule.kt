package org.team9432.resources.swerve.module

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.Logger
import org.team9432.resources.swerve.DriveControlLoops.DRIVE_CLOSE_LOOP
import org.team9432.resources.swerve.DriveControlLoops.DRIVE_OPEN_LOOP
import org.team9432.resources.swerve.DriveControlLoops.STEER_CLOSE_LOOP
import org.team9432.resources.swerve.DriveTrainConstants
import org.team9432.resources.swerve.mapleswerve.utils1.MaplePIDController
import org.team9432.resources.swerve.mapleswerve.utils1.SwerveStateProjection

class SwerveModule(io: ModuleIO, private val name: String) {
    private val io: ModuleIO = io
    private val inputs: LoggedModuleIOInputs = LoggedModuleIOInputs()

    private val turnCloseLoop: PIDController
    private val driveCloseLoop: PIDController
    private var setPoint: SwerveModuleState

    /**
     * Returns the module positions received this cycle.
     */
    var odometryPositions: Array<SwerveModulePosition> = Array(1) { SwerveModulePosition() }
        private set

    init {
        turnCloseLoop = MaplePIDController(STEER_CLOSE_LOOP)
        driveCloseLoop = MaplePIDController(DRIVE_CLOSE_LOOP)

        setPoint = SwerveModuleState()
        turnCloseLoop.calculate(steerFacing.radians) // activate close loop controller
        io.setDriveBrake(true)
        io.setSteerBrake(true)

        periodic()
    }

    fun periodic() {
        updateOdometryPositions()
    }

    fun updateOdometryInputs() {
        io.updateInputs(inputs)
        Logger.processInputs("Drive/Module-$name", inputs)
    }

    private fun updateOdometryPositions() {
        odometryPositions = Array(inputs.odometryDriveWheelRevolutions.size) { i ->
            val positionMeters = driveWheelRevolutionsToMeters(inputs.odometryDriveWheelRevolutions.get(i))
            val angle: Rotation2d = inputs.odometrySteerPositions[i]
            SwerveModulePosition(positionMeters, angle)
        }
    }

    private fun runSteerCloseLoop() {
        turnCloseLoop.setpoint = setPoint.angle.radians
        io.setSteerPowerPercent(turnCloseLoop.calculate(steerFacing.radians))
    }

    private fun runDriveControlLoop() {
        val adjustSpeedSetpointMetersPerSec: Double = SwerveStateProjection.project(setPoint, steerFacing)
        io.setDriveVoltage(
            DRIVE_OPEN_LOOP.calculate(adjustSpeedSetpointMetersPerSec)
                    + driveCloseLoop.calculate(driveVelocityMetersPerSec, adjustSpeedSetpointMetersPerSec)
        )
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized state.
     */
    fun runSetPoint(state: SwerveModuleState): SwerveModuleState {
        this.setPoint = SwerveModuleState.optimize(state, steerFacing)

        runDriveControlLoop()
        runSteerCloseLoop()

        return this.setPoint
    }

    val steerFacing: Rotation2d
        /**
         * Returns the current turn angle of the module.
         */
        get() = inputs.steerFacing

    val steerVelocityRadPerSec: Double
        get() = inputs.steerVelocityRadPerSec

    val drivePositionMeters: Double
        /**
         * Returns the current drive position of the module in meters.
         */
        get() = driveWheelRevolutionsToMeters(inputs.driveWheelFinalRevolutions)

    private fun driveWheelRevolutionsToMeters(driveWheelRevolutions: Double): Double {
        return Units.rotationsToRadians(driveWheelRevolutions) * DriveTrainConstants.WHEEL_RADIUS_METERS
    }

    val driveVelocityMetersPerSec: Double
        /**
         * Returns the current drive velocity of the module in meters per second.
         */
        get() = driveWheelRevolutionsToMeters(inputs.driveWheelFinalVelocityRevolutionsPerSec)

    val latestPosition: SwerveModulePosition
        /**
         * Returns the module position (turn angle and drive position).
         */
        get() = SwerveModulePosition(drivePositionMeters, steerFacing)

    val measuredState: SwerveModuleState
        /**
         * Returns the module state (turn angle and drive velocity).
         */
        get() = SwerveModuleState(driveVelocityMetersPerSec, steerFacing)
}