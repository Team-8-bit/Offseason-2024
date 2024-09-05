package org.team9432.resources.swerve.mapleswerve.utils.CustomMaths

import edu.wpi.first.math.geometry.Rotation3d
import org.team9432.GIT_SHA
import java.util.*
import kotlin.math.*

object MapleCommonMath {
    /**
     * random object that generates random variables
     * the seed is the hash of GIT_SHA
     * this way when you do log-replay even the generated random numbers are the same
     */
    private val random: Random = Random(GIT_SHA.hashCode().toLong())

    fun linearInterpretationWithBounding(x1: Double, y1: Double, x2: Double, y2: Double, x: Double): Double {
        val minX: Double = min(x1, x2)
        val maxX: Double = max(x1, x2)
        return linearInterpretation(x1, y1, x2, y2, min(maxX, max(minX, x)))
    }

    fun linearInterpretation(x1: Double, y1: Double, x2: Double, y2: Double, x: Double): Double {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1)
    }

    /**
     * using the random number generator of a fixed seed, generate the next random normal variable
     *
     * @param mean   the center of the distribution
     * @param stdDev the standard deviation of the distribution
     * @return the next random variable x from the distribution
     */
    fun generateRandomNormal(mean: Double, stdDev: Double): Double {
        val u1: Double = random.nextDouble()
        val u2: Double = random.nextDouble()
        // Box–Muller transform https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
        val z0: Double = sqrt(-2.0 * ln(u1)) * cos(2.0 * Math.PI * u2)
        return z0 * stdDev + mean
    }

    fun constrainMagnitude(value: Double, maxMagnitude: Double): Double {
        return min(
            abs(value),
            abs(maxMagnitude)
        ).withSign(
            value
        )
    }

    fun printRotation3d(rotation3d: Rotation3d): String {
        return "rotation 3d object with value: " + rotation3d.getQuaternion() + String.format(
            "\nand roll %.2f deg, pitch %.2f deg, yaw %.2f deg",
            Math.toDegrees(rotation3d.getX()), Math.toDegrees(rotation3d.getY()), Math.toDegrees(rotation3d.getZ())
        )
    }
}
