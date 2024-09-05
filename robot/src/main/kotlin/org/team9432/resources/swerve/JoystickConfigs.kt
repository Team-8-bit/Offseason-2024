package org.team9432.resources.swerve

/**
 * Configs for the driver's joystick
 * See [frc.robot.utils.MapleJoystickDriveInput]
 */
object JoystickConfigs {
    const val DEFAULT_TRANSLATIONAL_SENSITIVITY: Double = 1.0
    const val DEFAULT_ROTATIONAL_SENSITIVITY: Double = 0.7

    /** the amount of time that the chassis waits after the pilot's last input, before it places all the swerve wheels to standby-state (facing forward)  */
    const val NON_USAGE_TIME_RESET_WHEELS: Double = 1.0

    /**   */
    const val DEAD_BAND_WHEN_OTHER_AXIS_EMPTY: Double = 0.02
    const val DEAD_BAND_WHEN_OTHER_AXIS_FULL: Double = 0.1
    const val LINEAR_SPEED_INPUT_EXPONENT: Double = 1.6
    const val ROTATION_SPEED_INPUT_EXPONENT: Double = 2.0

    /**
     * the amount of time that the chassis needs to shift to the desired pilot motion
     * it's sort of a "smooth out" of the pilot's input
     * this dramatically reduces over-current and brownouts
     */
    const val LINEAR_ACCELERATION_SMOOTH_OUT_SECONDS: Double = 0.1

    /** same thing for rotation  */
    const val ANGULAR_ACCELERATION_SMOOTH_OUT_SECONDS: Double = 0.1

    /** the amount of time that the chassis waits after the pilot's last rotational input, before it starts to "lock" its rotation with PID  */
    const val TIME_ACTIVATE_ROTATION_MAINTENANCE_AFTER_NO_ROTATIONAL_INPUT_SECONDS: Double = 0.6
}