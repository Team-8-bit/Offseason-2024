@file:JvmName("Main") // set the compiled Java class name to "Main" rather than "MainKt"
package org.team9432

import com.choreo.lib.Choreo
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import kotlinx.coroutines.delay
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team9432.auto.AutoChooser
import org.team9432.lib.Library
import org.team9432.lib.coroutines.LoggedCoroutineRobot
import org.team9432.lib.coroutines.Team8BitRobot.Runtime.*
import org.team9432.lib.coroutines.robotPeriodic
import org.team9432.lib.util.ChoreoUtil.getAutoFlippedInitialPose
import org.team9432.lib.util.applyFlip
import org.team9432.oi.Controls
import org.team9432.resources.intake.Intake
import org.team9432.resources.loader.Loader
import org.team9432.resources.shooter.Shooter
import org.team9432.resources.swerve.Swerve
import org.team9432.vision.Vision
import kotlin.time.Duration.Companion.seconds


object Robot: LoggedCoroutineRobot() {
    val runtime = if (RobotBase.isReal()) REAL else SIM

    override suspend fun init() {
        Logger.recordMetadata("ProjectName", "2024-Offseason") // Set a metadata value
        Logger.recordMetadata("GIT_SHA", GIT_SHA)
        Logger.recordMetadata("GIT_DATE", GIT_DATE)
        Logger.recordMetadata("GIT_BRANCH", GIT_BRANCH)
        Logger.recordMetadata("BUILD_DATE", BUILD_DATE)
        Logger.recordMetadata("DIRTY", if (DIRTY == 1) "true" else "false")

        when (runtime) {
            REAL -> {
                Logger.addDataReceiver(WPILOGWriter()) // Log to a USB stick ("/U/logs")
                Logger.addDataReceiver(NT4Publisher()) // Publish data to NetworkTables
                PowerDistribution(1, PowerDistribution.ModuleType.kRev) // Enables power distribution logging
            }

            SIM -> {
                Logger.addDataReceiver(NT4Publisher())
                PowerDistribution(1, PowerDistribution.ModuleType.kRev) // Enables power distribution logging
            }

            REPLAY -> {
                setUseTiming(false) // Run as fast as possible
                val logPath = LogFileUtil.findReplayLog() // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(WPILOGReader(logPath)) // Read replay log
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay"))) // Save outputs to a new log
            }
        }

        Logger.start() // Start logging! No more data receivers, replay sources, or metadata values may be added.

        // Disables protobuf encoding warning, supposedly it's only bad the first few times it's called, we'll have to see
        // Original warning:
        // Warning at org.littletonrobotics.junction.LogTable.put(LogTable.java:429): Logging value to field "/Vision/Results" using protobuf encoding. This may cause high loop overruns, please monitor performance or save the value in a different format. Call "LogTable.disableProtobufWarning()" to disable this message.
        LogTable.disableProtobufWarning()

        Library.initialize(this, runtime)

        Swerve
        Intake
        Shooter
        Loader

        Controls
        Vision
        Beambreaks

        NoteVisualizer

        PortForwarder.add(5800, "10.94.32.11", 5800)
        PortForwarder.add(5800, "10.94.32.12", 5800)

        `LEDs!`

        AutoChooser

        DriverStation.silenceJoystickConnectionWarning(true)
    }


    override suspend fun teleop() {
        robotPeriodic(isFinished = { !Robot.mode.isTeleop }) {
            val speeds = Controls.getTeleopSwerveRequest()
            Swerve.runFieldCentricChassisSpeeds(speeds)
        }
    }

    override suspend fun autonomous() {
        RobotController.setAction {
            val selectedAuto = AutoChooser.getAuto()

            if (selectedAuto == null) {
                println("[Error] Auto was null")
                return@setAction
            }

            when (selectedAuto) {
                is FourNote -> RobotFourNote.run(selectedAuto)
                is FarsideCenterline -> RobotFarsideCenterline.run(selectedAuto)
                is AmpsideCenterline -> RobotAmpsideCenterline.run(selectedAuto)
            }
        }
    }

    override suspend fun disabled() {
        RobotController.resetRequests()
    }

    override suspend fun test() {
//        RobotController.setAction {
//            wheelDiameterTest(rotationsPerSecond = 0.125)
//        }
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
fun main() {
    RobotBase.startRobot { Robot }
}