package org.team9432.resources.swerve

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields

object VisionConfigs {
    val fieldLayout: AprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField()
    const val MINIMUM_TAGS_NUM: Int = 2

    /* default standard error for vision observation, if only one apriltag observed */
    const val TRANSLATIONAL_STANDARD_ERROR_METERS_FOR_SINGLE_OBSERVATION: Double = 0.3
    val ROTATIONAL_STANDARD_ERROR_RADIANS_FOR_SINGLE_OBSERVATION: Double = Math.toRadians(10.0)

    // only do odometry calibration if translational standard error if it is not greater than
    const val TRANSLATIONAL_STANDARD_ERROR_THRESHOLD: Double = 0.5

    // only do gyro calibration if rotational standard error is very, very small
    val ROTATIONAL_STANDARD_ERROR_THRESHOLD: Double = Math.toRadians(5.0)
    val ROTATIONAL_ERROR_WITH_GYRO_DISCARD_RESULT: Double = Math.toRadians(15.0)

    const val ODOMETRY_TRANSLATIONAL_STANDARD_ERROR_METERS: Double = 0.02

    // we trust the IMU very much (recommend 0.1 for Pigeon2, 0.5 for NavX)
    val GYRO_ROTATIONAL_STANDARD_ERROR_RADIANS: Double = Math.toRadians(0.1)
}