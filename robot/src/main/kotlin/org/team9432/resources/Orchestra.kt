package org.team9432.resources

import com.ctre.phoenix6.Orchestra
import org.team9432.resources.swerve.Swerve

object Orchestra {
    private val orchestra = Orchestra()

    init {
        orchestra.addInstrument(Swerve.swerve.getModule(0).driveMotor)
        orchestra.addInstrument(Swerve.swerve.getModule(0).steerMotor)
        orchestra.addInstrument(Swerve.swerve.getModule(1).driveMotor)
        orchestra.addInstrument(Swerve.swerve.getModule(1).steerMotor)
        orchestra.addInstrument(Swerve.swerve.getModule(2).driveMotor)
        orchestra.addInstrument(Swerve.swerve.getModule(2).steerMotor)
        orchestra.addInstrument(Swerve.swerve.getModule(3).driveMotor)
        orchestra.addInstrument(Swerve.swerve.getModule(3).steerMotor)
    }

    fun play(track: String) {
        orchestra.loadMusic(track)
        orchestra.play()
    }
}