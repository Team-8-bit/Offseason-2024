package org.team9432.resources.swerve

import edu.wpi.first.math.util.Units
import org.team9432.lib.coroutines.robotPeriodic
import org.team9432.lib.doglog.Logger
import org.team9432.lib.unit.inRadians
import kotlin.math.abs
import kotlin.math.hypot

// https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/commands/WheelRadiusCharacterization.java
suspend fun wheelDiameterTest(rotationsPerSecond: Double) {
    val drivetrainRadiusMeters = Units.inchesToMeters(hypot(9.875, 6.875))

    val initialHeading = Swerve.getGyroHeading()
    val initialWheelPositions = Swerve.getModuleWheelPositionsRadians()

    robotPeriodic(isFinished = { false }) {
        Swerve.setWheelCharacterizationDriveControl(rotationsPerSecond)

        val accumulatedHeading = Swerve.getGyroHeading().inRadians - initialHeading.inRadians
        val avgAccumulatedWheelPosition =
            Swerve.getModuleWheelPositionsRadians()
                .zip(initialWheelPositions)
                .map { (currentPosition, initialPosition) -> abs(currentPosition - initialPosition) }
                .average()

        val effectiveRadius = abs((accumulatedHeading * drivetrainRadiusMeters) / avgAccumulatedWheelPosition)

        Logger.log("Swerve/EffectiveWheelDiameter", Units.metersToInches(effectiveRadius * 2))
    }
}