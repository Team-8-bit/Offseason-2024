package org.team9432.resources.swerve.mapleswerve.utils.CustomMaths

import kotlin.math.abs
import kotlin.math.withSign

object Angles {
    /**
     * simplify an angle into the range 0-360 degrees
     *
     * @param radian the angle to simplify, in radian
     * @return the simplified angle, in radian and in the range 0 < x < Math.Pi*2
     */
    fun simplifyAngle(radian: Double): Double {
        var radian: Double = radian
        require(!(java.lang.Double.isNaN(radian) || java.lang.Double.isInfinite(radian) || abs(radian) > 10e7)) { "invalid radian: " + radian }
        radian = (radian % (Math.PI * 2)).withSign(radian)
        if (radian < 0) radian += Math.PI * 2
        return radian
    }

    /**
     * gets the shortest rotational distance(and its direction) needed to get from the current to
     * targeted rotation
     *
     * @param currentRotation  the current rotation, in radian
     * @param targetedRotation the desired rotation, in radian
     * @return the shortest distance between the two points, in radian and positive is
     * counter-clockwise
     */
    fun getActualDifference(currentRotation: Double, targetedRotation: Double): Double {
        var currentRotation: Double = currentRotation
        var targetedRotation: Double = targetedRotation
        val loopLength: Double = Math.PI * 2
        currentRotation = simplifyAngle(currentRotation)
        targetedRotation = simplifyAngle(targetedRotation)
        val difference: Double = targetedRotation - currentRotation
        if (difference > loopLength / 2) return -(loopLength - difference) // go the other way around

        if (difference < -loopLength / 2) return loopLength + difference // go the other way around

        return difference
    }

    /**
     * get the mid point between two points
     */
    fun findMidPoint(rotation1: Double, rotation2: Double): Double {
        return simplifyAngle(rotation1 + getActualDifference(rotation1, rotation2))
    }
}
