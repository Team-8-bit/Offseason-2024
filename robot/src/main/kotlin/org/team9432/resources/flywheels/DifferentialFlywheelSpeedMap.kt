package org.team9432.resources.flywheels

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import org.team9432.lib.unit.Length
import org.team9432.lib.unit.inMeters

class DifferentialFlywheelSpeedMap {
    private val topShooterMap = InterpolatingDoubleTreeMap()
    private val bottomShooterMap = InterpolatingDoubleTreeMap()

    fun addMapValue(distance: Length, speeds: ShooterSpeeds) {
        topShooterMap.put(distance.inMeters, speeds.upperRPM)
        bottomShooterMap.put(distance.inMeters, speeds.lowerRPM)
    }

    fun getMapValue(distance: Length): ShooterSpeeds {
        val topSpeed = topShooterMap.get(distance.inMeters)
        val bottomSpeed = bottomShooterMap.get(distance.inMeters)
        return ShooterSpeeds(topSpeed, bottomSpeed)
    }

    data class ShooterSpeeds(val upperRPM: Double, val lowerRPM: Double) {
        val isIdle = upperRPM == 0.0 && lowerRPM == 0.0
    }
}