package org.team9432.resources.swerve.mapleswerve.utils1

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import kotlin.math.cos

object SwerveStateProjection {
    /**
     * Projects the swerve module speed onto the direction of the current swerve facing.
     * @param swerveSpeed The current speed and direction of the swerve module.
     * @param currentSwerveFacing The desired direction to project onto.
     * @return The projected speed in the direction of currentSwerveFacing.
     */
    fun project(swerveSpeed: SwerveModuleState, currentSwerveFacing: Rotation2d?): Double {
        // Get the angle of the swerve module's current direction
        val swerveModuleAngle = swerveSpeed.angle

        // Calculate the cosine of the angle difference between swerve module direction and the desired direction
        val cosTheta = cos(swerveModuleAngle.minus(currentSwerveFacing).radians)

        // Scale the speed by the cosine value to get the projection
        return swerveSpeed.speedMetersPerSecond * cosTheta
    }
}