package org.team9432.auto.paths

import org.team9432.choreogenerator.GeneratorFile
import org.team9432.choreogenerator.json.ChoreoRobotConfiguration
import org.team9432.lib.unit.inMeters
import org.team9432.lib.unit.inches
import java.io.File
import kotlin.system.measureTimeMillis

val OSR2024Config = ChoreoRobotConfiguration(
    mass = 74.08797700309194,
    rotationalInertia = 6,
    motorMaxTorque = 1.162295081967213,
    motorMaxVelocity = 4800,
    gearing = 5.9,
//    wheelbase = 13.75.inches.inMeters,
    wheelbase = 19.75.inches.inMeters,
    trackWidth = 19.75.inches.inMeters,
//    bumperLength = 25.75.inches.inMeters,
    bumperLength = 32.inches.inMeters,
    bumperWidth = 32.inches.inMeters,
    wheelRadius = 1.92.inches.inMeters
)

fun main() {
    val outputFile = File("output.chor")

    val time = measureTimeMillis {
        val file = GeneratorFile(OSR2024Config)
        FourNote.getAllPossibilities().forEach { file.addPath(it) }
        outputFile.writeText(file.export())
    }

    println("Generated file ${outputFile.absolutePath} in ${time}ms")
}