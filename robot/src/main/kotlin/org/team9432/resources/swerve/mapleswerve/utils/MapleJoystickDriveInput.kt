package org.team9432.resources.swerve.mapleswerve.utils

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.team9432.resources.swerve.JoystickConfigs.DEAD_BAND_WHEN_OTHER_AXIS_EMPTY
import org.team9432.resources.swerve.JoystickConfigs.DEAD_BAND_WHEN_OTHER_AXIS_FULL
import org.team9432.resources.swerve.JoystickConfigs.LINEAR_SPEED_INPUT_EXPONENT
import org.team9432.resources.swerve.JoystickConfigs.ROTATION_SPEED_INPUT_EXPONENT
import org.team9432.resources.swerve.mapleswerve.utils.CustomMaths.MapleCommonMath
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.withSign

/**
 * Some optimizations to the pilot's input, including a linear dead band and
 */
class MapleJoystickDriveInput(joystickXSupplier: DoubleSupplier, joystickYSupplier: DoubleSupplier, joystickOmegaSupplier: DoubleSupplier) {
    val joystickXSupplier: DoubleSupplier
    val joystickYSupplier: DoubleSupplier
    val joystickOmegaSupplier: DoubleSupplier

    /**
     * @param joystickXSupplier the supplier of the x-axis of the joystick, positive is RIGHTWARDS
     * @param joystickYSupplier the supplier of the x-axis of the joystick, positive is DOWNWARDS
     * @param joystickOmegaSupplier the supplier of the omega-axis of the joystick, positive is RIGHTWARDS
     */
    init {
        this.joystickXSupplier = joystickXSupplier
        this.joystickYSupplier = joystickYSupplier
        this.joystickOmegaSupplier = joystickOmegaSupplier
    }

    /**
     * reads the joystick inputs and calculate the chassis speeds
     * @return the chassis speeds requested by the pilot, driver-station-centric
     */
    fun getJoystickChassisSpeeds(chassisMaxVelocityMetersPerSec: Double, maxAngularVelocityRadPerSec: Double): ChassisSpeeds {
        val linearSpeedMetersPerSec: Translation2d = getTranslationalSpeedsFromJoystick(chassisMaxVelocityMetersPerSec)
        val rotationSpeedRadPerSec: Double = getRotationalSpeedFromJoystick(maxAngularVelocityRadPerSec)

        return ChassisSpeeds(linearSpeedMetersPerSec.getX(), linearSpeedMetersPerSec.getY(), rotationSpeedRadPerSec)
    }

    /**
     * @return the translational speeds, in meters/second
     */
    fun getTranslationalSpeedsFromJoystick(chassisMaxVelocityMetersPerSec: Double): Translation2d {
        val linearSpeedXComponentRaw: Double = -joystickYSupplier.getAsDouble()
        val linearSpeedYComponentRaw: Double = -joystickXSupplier.getAsDouble()
        val linearSpeedXComponentDeadBanded: Double = applySmartDeadBand(linearSpeedXComponentRaw, linearSpeedYComponentRaw)
        val linearSpeedYComponentDeadBanded: Double = applySmartDeadBand(linearSpeedYComponentRaw, linearSpeedXComponentRaw)

        val originalTranslationalSpeed: Translation2d = Translation2d(linearSpeedXComponentDeadBanded, linearSpeedYComponentDeadBanded)
        val translationalSpeedMagnitudeScaled: Double = originalTranslationalSpeed.getNorm().pow(LINEAR_SPEED_INPUT_EXPONENT)
        return Translation2d(
            translationalSpeedMagnitudeScaled * chassisMaxVelocityMetersPerSec,
            originalTranslationalSpeed.getAngle()
        )
    }

    fun getRotationalSpeedFromJoystick(maxAngularVelocityRadPerSec: Double): Double {
        val rotationSpeedRaw: Double = -joystickOmegaSupplier.getAsDouble()
        val rotationalSpeedDeadBanded: Double = applySmartDeadBand(rotationSpeedRaw, 0.0)
        val rotationalSpeedScaledMagnitude: Double = abs(
            rotationalSpeedDeadBanded.pow(
                ROTATION_SPEED_INPUT_EXPONENT
            )
        ) * maxAngularVelocityRadPerSec
        return rotationalSpeedScaledMagnitude.withSign(rotationSpeedRaw)
    }

    companion object {
        /**
         * apply  a smart dead-band to the given axis value
         * unlike normal dead-banding, the threshold of a smart deadband increases as the value of the other axis increases
         * this will make it easier for the pilot to set request a straight-line driving
         * @param axisValue the value of the axis of interest
         * @param otherAxisValue the value of the other axis on the stick
         */
        private fun applySmartDeadBand(axisValue: Double, otherAxisValue: Double): Double {
            val deadBand: Double = MapleCommonMath.linearInterpretationWithBounding(
                0.0, DEAD_BAND_WHEN_OTHER_AXIS_EMPTY,
                1.0, DEAD_BAND_WHEN_OTHER_AXIS_FULL,
                abs(otherAxisValue)
            )
            return MathUtil.applyDeadband(axisValue, deadBand, 1.0)
        }

        fun leftHandedJoystick(driverController: CommandXboxController): MapleJoystickDriveInput {
            return MapleJoystickDriveInput(
                DoubleSupplier { driverController.getLeftX() },
                DoubleSupplier { driverController.getLeftY() },
                DoubleSupplier { driverController.getRightX() }
            )
        }

        fun leftHandedJoystick(driverController: XboxController): MapleJoystickDriveInput {
            return MapleJoystickDriveInput(
                DoubleSupplier { driverController.getLeftX() },
                DoubleSupplier { driverController.getLeftY() },
                DoubleSupplier { driverController.getRightX() }
            )
        }

        fun rightHandedJoystick(driverController: CommandXboxController): MapleJoystickDriveInput {
            return MapleJoystickDriveInput(
                DoubleSupplier { driverController.getRightX() },
                DoubleSupplier { driverController.getRightY() },
                DoubleSupplier { driverController.getLeftX() }
            )
        }

        fun rightHandedJoystick(driverController: XboxController): MapleJoystickDriveInput {
            return MapleJoystickDriveInput(
                DoubleSupplier { driverController.getRightX() },
                DoubleSupplier { driverController.getRightY() },
                DoubleSupplier { driverController.getLeftX() }
            )
        }
    }
}
