package org.team9432.auto.paths

import org.team9432.auto.types.AutoType
import org.team9432.choreogenerator.ChoreoFile
import org.team9432.choreogenerator.ChoreoRobotConfiguration
import org.team9432.lib.unit.inches
import java.io.File
import kotlin.system.measureTimeMillis

val OSR2024Config = ChoreoRobotConfiguration(
    mass = 74.08797700309194,
    rotationalInertia = 6,
    motorMaxTorque = 1.162295081967213,
    motorMaxVelocity = 4800,
    gearing = 5.9,
//    wheelbase = 13.75.inches,
    wheelbase = 19.75.inches,
    trackWidth = 19.75.inches,
//    bumperLength = 25.75.inches,
    bumperLength = 32.inches,
    bumperWidth = 32.inches,
    wheelRadius = 1.92.inches
)

fun main() {
    val outputFile = File("output.chor") // This magically appears in the right place, but I don't know why

    val time = measureTimeMillis {
        val choreoFile = ChoreoFile(outputFile, OSR2024Config, splitTrajectoriesAtStopPoints = true)

        AutoType.FourNote.options.forEach { choreoFile.addPath(FourNotePaths.generateFourNote(it)) }

        choreoFile.outputToFile()
    }

    println("Generated file ${outputFile.absolutePath} in ${time}ms")
}