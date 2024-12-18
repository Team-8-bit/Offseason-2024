package org.team9432.resources.drive.module

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import org.littletonrobotics.junction.Logger
import org.team9432.lib.dashboard.LoggedTunableNumber
import org.team9432.resources.drive.DrivetrainConstants.WHEEL_RADIUS_METERS
import org.team9432.resources.drive.TunerConstants


class SwerveModule(private val io: ModuleIO, private val name: String) {
    private val inputs: LoggedModuleIOInputs = LoggedModuleIOInputs()

    private var angleSetpoint: Rotation2d? = null
    private var speedSetpoint: Double? = null

    private var driveFeedforward = SimpleMotorFeedforward(0.0, 0.0)

    var odometryPositions: Array<SwerveModulePosition> = emptyArray()
        private set

    companion object {
        val drivekP: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/DrivekP", 35.0)
        val drivekD: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/DrivekD", 0.0)
        val drivekS: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/DrivekS", 5.0)
        val drivekV: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/DrivekV", 0.0)
        val kT = 1.0 / DCMotor.getKrakenX60Foc(1).KtNMPerAmp
        val steerkP: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/SteerkP", TunerConstants.steerGains.kP)
        val steerkD: LoggedTunableNumber = LoggedTunableNumber("Drive/Module/SteerkD", TunerConstants.steerGains.kD)
    }

    init {
        io.setDriveBrake(true)
        io.setSteerBrake(true)
    }

    fun updateInputs() = io.updateInputs(inputs)

    fun periodic() {
        Logger.processInputs("Drive/Module-$name", inputs)

        LoggedTunableNumber.ifChanged(
            hashCode(),
            { driveFeedforward = SimpleMotorFeedforward(drivekS.get(), drivekV.get(), 0.0) },
            drivekS,
            drivekV
        )
        LoggedTunableNumber.ifChanged(
            hashCode(), { io.setDrivePID(drivekP.get(), 0.0, drivekD.get()) }, drivekP, drivekD
        )
        LoggedTunableNumber.ifChanged(
            hashCode(), { io.setSteerPID(steerkP.get(), 0.0, steerkD.get()) }, steerkP, steerkD
        )

        trackClosedLoopStates()

        odometryPositions = Array(inputs.odometryDrivePositionsRotations.size) { i ->
            val positionMeters = driveWheelRotationsToMeters(inputs.odometryDrivePositionsRotations[i])
            val angle: Rotation2d = inputs.odometrySteerPositions[i]
            SwerveModulePosition(positionMeters, angle)
        }
    }

    fun runCharacterization(turnSetpointDegrees: Double, input: Double) {
        io.runSteerPosition(Rotation2d.fromDegrees(turnSetpointDegrees))
        io.runCharacterization(input)
    }

    val characterizationVelocity get() = inputs.driveVelocityRadPerSecond

    private fun trackClosedLoopStates() {
//        val angleTarget = angleSetpoint
//        val speedTarget = speedSetpoint
//        // Run closed loop turn control
//        if (angleTarget != null) {
//            io.runSteerPosition(angleTarget)
//
//            // Run closed loop drive control
//            // Only if closed loop turn control is running
//            if (speedTarget != null) {
//                // Scale velocity based on turn error
//
//                // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
//                // towards the setpoint, its velocity should increase. This is achieved by
//                // taking the component of the velocity in the direction of the setpoint.
//                val angleAdjustedTarget = speedTarget * cos(angleTarget.minus(inputs.steerAbsolutePosition).radians)
//                val deadbandedTarget = MathUtil.applyDeadband(angleAdjustedTarget, .025)
//
//                // Run drive controller
//                io.runDriveVelocity(deadbandedTarget, simSwitch(real = 0.0, sim = driveFeedforward.calculate(deadbandedTarget / WHEEL_RADIUS_METERS)))
//            }
//        }
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized state.
     */
    fun runSetpoint(setpoint: SwerveModuleState, torqueFF: SwerveModuleState) {
        this.angleSetpoint = setpoint.angle
        this.speedSetpoint = setpoint.speedMetersPerSecond

        val wheelTorqueNm = torqueFF.speedMetersPerSecond
        io.runDriveVelocitySetpoint(
            setpoint.speedMetersPerSecond / Units.inchesToMeters(TunerConstants.kWheelRadiusInches),
            driveFeedforward.calculate(setpoint.speedMetersPerSecond / Units.inchesToMeters(TunerConstants.kWheelRadiusInches))
                    + ((wheelTorqueNm / TunerConstants.kDriveGearRatio) * kT)
        )

        io.runSteerPosition(setpoint.angle)
    }

    /** The current turn angle of the module. */
    val angle: Rotation2d
        get() = inputs.steerAbsolutePosition

    val steerVelocityRadPerSec: Double
        get() = inputs.steerVelocityRadPerSec

    val drivePositionMeters: Double
        /**
         * Returns the current drive position of the module in meters.
         */
        get() = driveWheelRotationsToMeters(inputs.drivePositionRotations)

    private fun driveWheelRotationsToMeters(driveWheelRotations: Double): Double {
        return Units.rotationsToRadians(driveWheelRotations) * WHEEL_RADIUS_METERS
    }

    val drivePositionRads: Double
        get() = Units.rotationsToRadians(inputs.drivePositionRotations)

    val driveVelocityMetersPerSec: Double
        /**
         * Returns the current drive velocity of the module in meters per second.
         */
        get() = driveWheelRotationsToMeters(Units.radiansToRotations(inputs.driveVelocityRadPerSecond))

    val latestPosition: SwerveModulePosition
        /**
         * Returns the module position (turn angle and drive position).
         */
        get() = SwerveModulePosition(drivePositionMeters, angle)

    val measuredState: SwerveModuleState
        /**
         * Returns the module state (turn angle and drive velocity).
         */
        get() = SwerveModuleState(driveVelocityMetersPerSec, angle)
}