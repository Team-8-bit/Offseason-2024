package org.team9432.resources.swerve.gyro

import edu.wpi.first.math.geometry.Rotation2d
import org.team9432.annotation.Logged

interface GyroIO {
    @Logged
    open class GyroIOInputs {
        var connected: Boolean = false
        var yawPosition: Rotation2d = Rotation2d()
        var odometryYawPositions: Array<Rotation2d> = arrayOf()
        var yawVelocityRadPerSec: Double = 0.0
    }

    fun updateInputs(inputs: GyroIOInputs)
}