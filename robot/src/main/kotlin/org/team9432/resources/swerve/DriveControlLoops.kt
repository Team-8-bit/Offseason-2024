package org.team9432.resources.swerve

import edu.wpi.first.math.controller.SimpleMotorFeedforward
import org.team9432.resources.swerve.DriveTrainConstants.DRIVE_FRICTION_VOLTAGE
import org.team9432.resources.swerve.mapleswerve.utils1.MaplePIDController.MaplePIDConfig

object DriveControlLoops {
    val CHASSIS_ROTATION_CLOSE_LOOP: MaplePIDConfig = MaplePIDConfig(
        Math.toRadians(360.0),
        Math.toRadians(60.0),
        0.02,
        Math.toRadians(3.0),
        0.15,
        true,
        0.0
    )

    val CHASSIS_TRANSLATION_CLOSE_LOOP: MaplePIDConfig = MaplePIDConfig(
        2.0,
        1.2,
        0.0,
        0.03,
        0.0,
        false,
        0.0
    )
    val STEER_CLOSE_LOOP: MaplePIDConfig = MaplePIDConfig(
        0.5,
        Math.toRadians(90.0),
        0.02,
        Math.toRadians(1.5),
        0.0,
        true,
        0.0
    )

    val DRIVE_OPEN_LOOP: SimpleMotorFeedforward = SimpleMotorFeedforward(
        DRIVE_FRICTION_VOLTAGE,
        12 / DriveTrainConstants.CHASSIS_MAX_VELOCITY
    )
    val DRIVE_CLOSE_LOOP: MaplePIDConfig = MaplePIDConfig(
        5.0,
        2.0,
        0.0,
        0.0,
        0.0,
        false, 0.0
    )
}