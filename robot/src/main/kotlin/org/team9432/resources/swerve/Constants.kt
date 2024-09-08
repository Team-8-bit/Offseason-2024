package org.team9432.resources.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import org.team9432.lib.simulation.RobotSimulationProfile
import org.team9432.lib.unit.Translation2d
import org.team9432.lib.unit.inches
import org.team9432.lib.unit.meters
import kotlin.math.min

/**
 * stores the constants and PID configs for chassis
 * because we want an all-real simulation for the chassis, the numbers are required to be considerably precise
 */
object DriveTrainConstants {
    val ROBOT_TO_BUMPER_CENTER_OFFSET = Transform2d(Units.inchesToMeters(2.75), 0.0, Rotation2d())

    /**
     * numbers that needs to be changed to fit each robot
     * TODO: change these numbers to match your robot
     */
    const val WHEEL_COEFFICIENT_OF_FRICTION: Double = 0.95
    const val ROBOT_MASS_KG: Double = 60.0 // with bumpers

    /**
     * TODO: change motor type to match your robot
     */
    val DRIVE_MOTOR: DCMotor = DCMotor.getKrakenX60(1)
    val STEER_MOTOR: DCMotor = DCMotor.getKrakenX60(1)

    /**
     * numbers imported from [TunerConstants]
     */
    val WHEEL_RADIUS_METERS: Double = Units.inchesToMeters(TunerConstants.kWheelRadiusInches)
    const val DRIVE_GEAR_RATIO: Double = TunerConstants.kDriveGearRatio
    const val STEER_GEAR_RATIO: Double = TunerConstants.kSteerGearRatio
    const val TIME_ROBOT_STOP_ROTATING_SECONDS: Double = 0.06
    const val STEER_FRICTION_VOLTAGE: Double = TunerConstants.kSteerFrictionVoltage
    const val DRIVE_FRICTION_VOLTAGE: Double = TunerConstants.kDriveFrictionVoltage
    const val STEER_INERTIA: Double = 0.01

    /* adjust current limit */
    const val DRIVE_CURRENT_LIMIT: Double = 60.0
    const val STEER_CURRENT_LIMIT: Double = 20.0


    /**
     * translations of the modules to the robot center, in FL, FR, BL, BR
     */
    val MODULE_TRANSLATIONS = listOf(
        Translation2d(
            TunerConstants.kFrontLeftXPosInches.inches,
            TunerConstants.kFrontLeftYPosInches.inches
        ),
        Translation2d(
            TunerConstants.kFrontRightXPosInches.inches,
            TunerConstants.kFrontRightYPosInches.inches
        ),
        Translation2d(
            TunerConstants.kBackLeftXPosInches.inches,
            TunerConstants.kBackLeftYPosInches.inches
        ),
        Translation2d(
            TunerConstants.kBackRightXPosInches.inches,
            TunerConstants.kBackRightYPosInches.inches
        )
    )

    /* equations that calculates some constants for the simulator (don't modify) */
    private const val GRAVITY_CONSTANT = 9.8
    val DRIVE_BASE_RADIUS: Double = MODULE_TRANSLATIONS[0].norm

    /* friction_force = normal_force * coefficient_of_friction */
    const val MAX_FRICTION_ACCELERATION: Double = GRAVITY_CONSTANT * WHEEL_COEFFICIENT_OF_FRICTION

    /* force = torque / distance */
    val MAX_PROPELLING_FORCE_NEWTONS: Double = DRIVE_MOTOR.getTorque(DRIVE_CURRENT_LIMIT) * DRIVE_GEAR_RATIO / WHEEL_RADIUS_METERS

    /* floor_speed = wheel_angular_velocity * wheel_radius */
    val CHASSIS_MAX_VELOCITY: Double = DRIVE_MOTOR.freeSpeedRadPerSec / DRIVE_GEAR_RATIO * WHEEL_RADIUS_METERS // calculate using choreo
    val CHASSIS_MAX_ACCELERATION_MPS_SQ: Double = min(
        MAX_FRICTION_ACCELERATION,  // cannot exceed max friction
        MAX_PROPELLING_FORCE_NEWTONS / ROBOT_MASS_KG
    )
    val CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC: Double = CHASSIS_MAX_VELOCITY / DRIVE_BASE_RADIUS
    val CHASSIS_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ: Double = CHASSIS_MAX_ACCELERATION_MPS_SQ / DRIVE_BASE_RADIUS
    val CHASSIS_FRICTIONAL_ANGULAR_ACCELERATION: Double = CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC / TIME_ROBOT_STOP_ROTATING_SECONDS

    val DRIVE_KINEMATICS: SwerveDriveKinematics = SwerveDriveKinematics(*MODULE_TRANSLATIONS.toTypedArray())

    /* for collision detection in simulation */
    val BUMPER_WIDTH_METERS: Double = Units.inchesToMeters(32.0)
    val BUMPER_LENGTH_METERS: Double = Units.inchesToMeters(31.5)

    /* Gyro Sim */
    const val GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ: Double = 100.0
    val SKIDDING_AMOUNT_AT_THRESHOLD_RAD: Double = Math.toRadians(1.2)

    /*
     * https://store.ctr-electronics.com/pigeon-2/
     * for a well-installed one with vibration reduction, only 0.4 degree
     * but most teams just install it directly on the rigid chassis frame (including my team :D)
     * so at least 1.2 degrees of drifting in 1 minutes for an average angular velocity of 60 degrees/second
     * which is the average velocity during normal swerve-circular-offense
     * */
    val NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD: Double = Math.toRadians(1.2)
    val AVERAGE_VELOCITY_RAD_PER_SEC_DURING_TEST: Double = Math.toRadians(60.0)

    const val ODOMETRY_FREQUENCY: Double = 250.0
    const val ODOMETRY_CACHE_CAPACITY: Int = 10
    const val ODOMETRY_WAIT_TIMEOUT_SECONDS: Double = 0.02

    val simProfile = RobotSimulationProfile(
        moduleTranslations = MODULE_TRANSLATIONS,
        wheelRadius = WHEEL_RADIUS_METERS.meters,
        kinematics = DRIVE_KINEMATICS,
        robotMaxVelocity = CHASSIS_MAX_VELOCITY,
        robotMaxAcceleration = CHASSIS_MAX_ACCELERATION_MPS_SQ,
        maxAngularVelocity = CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
        robotMass = ROBOT_MASS_KG,
        width = BUMPER_WIDTH_METERS,
        height = BUMPER_LENGTH_METERS,
        robotBumperToCenterOffset = ROBOT_TO_BUMPER_CENTER_OFFSET,
        frictionForce = MAX_FRICTION_ACCELERATION * ROBOT_MASS_KG,
        angularFrictionAcceleration = CHASSIS_FRICTIONAL_ANGULAR_ACCELERATION
    )
}