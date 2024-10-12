// By team 6328: https://github.com/Mechanical-Advantage/RobotCode2024/blob/a025615a52193b7709db7cf14c51c57be17826f2/src/main/java/org/littletonrobotics/frc2024/commands/WheelRadiusCharacterization.java

package org.team9432.resources.drive

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import org.team9432.RobotState
import org.team9432.lib.dashboard.LoggedTunableNumber
import kotlin.math.abs


class WheelRadiusCharacterization(private val drive: Drive, private val direction: Direction): Command() {
    private val driveRadius = DrivetrainConstants.DRIVE_BASE_RADIUS

    private val omegaLimiter = SlewRateLimiter(1.0)

    private var lastGyroYawRads = 0.0
    private var accumGyroYawRads = 0.0

    private var startWheelPositions = List(4) { 0.0 }

    private var currentEffectiveWheelRadius = 0.0

    companion object {
        val characterizationSpeed by LoggedTunableNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 0.1)
    }

    enum class Direction(val value: Int) {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1)
    }

    init {
        addRequirements(drive)
    }

    private fun getYaw() = RobotState.currentPose.rotation.radians

    override fun initialize() {
        // Reset
        lastGyroYawRads = getYaw()
        accumGyroYawRads = 0.0

        startWheelPositions = drive.getWheelRadiusCharacterizationPositions()

        omegaLimiter.reset(0.0)
    }

    override fun execute() {
        // Run drive at velocity
        drive.runWheelRadiusCharacterization(
            omegaLimiter.calculate(direction.value * characterizationSpeed)
        )

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(getYaw() - lastGyroYawRads)
        lastGyroYawRads = getYaw()
        var averageWheelPosition = 0.0
        val wheelPositions = drive.getWheelRadiusCharacterizationPositions()
        for (i in 0..3) {
            averageWheelPosition += abs(wheelPositions[i] - startWheelPositions[i])
        }
        averageWheelPosition /= 4.0

        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition
        Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition)
        Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads)
        Logger.recordOutput(
            "Drive/RadiusCharacterization/CurrentWheelRadiusInches", Units.metersToInches(currentEffectiveWheelRadius)
        )
    }

    override fun end(interrupted: Boolean) {
        drive.endCharacterization()
        if (accumGyroYawRads <= Math.PI * 2.0) {
            println("Not enough data for characterization")
        } else {
            println("Effective Wheel Radius: ${Units.metersToInches(currentEffectiveWheelRadius)} inches")
        }
    }
}