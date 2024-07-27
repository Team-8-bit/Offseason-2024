@file:JvmName("Main") // set the compiled Java class name to "Main" rather than "MainKt"
package org.team9432

import edu.wpi.first.wpilibj.RobotBase
import kotlinx.coroutines.launch
import org.team9432.io.Buttons
import org.team9432.lib.led.animations.solid
import org.team9432.lib.led.color.Color
import org.team9432.lib.led.color.predefined.Red
import org.team9432.lib.led.management.AnimationManager
import org.team9432.lib.led.management.Section
import org.team9432.lib.led.strip.LEDStrip
import org.team9432.lib.led.strip.RioLedStrip
import org.team9432.lib.robot.CoroutineRobot
import org.team9432.lib.robot.RobotScope
import org.team9432.resources.Loader
import org.team9432.resources.Intake
import org.team9432.resources.Shooter
import org.team9432.resources.swerve.Swerve
import org.team9432.vision.PhotonVision
import kotlin.time.Duration.Companion.milliseconds


object Robot: CoroutineRobot() {
    override suspend fun init() {
        Logging.start()

        Intake
        Shooter
        Loader
        Swerve

        Buttons
        PhotonVision

//        LEDStrip.create(RioLedStrip(30, 0))
//
//        RobotScope.launch {
//            AnimationManager.run(20.milliseconds)
//        }
//
//        val leds = Section((0..29).toSet())
//
//        leds.solid(Color.Red)
    }

    override suspend fun autonomous() {
        Auto.runFourNote()
    }
}

/**
 * Main initialization function. Do not perform any initialization here
 * other than calling `RobotBase.startRobot`. Do not modify this file
 * except to change the object passed to the `startRobot` call.
 *
 * If you change the package of this file, you must also update the
 * `ROBOT_MAIN_CLASS` variable in the gradle build file. Note that
 * this file has a `@file:JvmName` annotation so that its compiled
 * Java class name is "Main" rather than "MainKt". This is to prevent
 * any issues/confusion if this file is ever replaced with a Java class.
 *
 * If you change your main Robot object (name), change the parameter of the
 * `RobotBase.startRobot` call to the new name. (If you use the IDE's Rename
 * Refactoring when renaming the object, it will get changed everywhere
 * including here.)
 */
fun main() = RobotBase.startRobot { Robot }