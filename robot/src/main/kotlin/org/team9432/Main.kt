@file:JvmName("Main") // set the compiled Java class name to "Main" rather than "MainKt"
package org.team9432

import choreo.auto.AutoChooser
import com.ctre.phoenix6.CANBus
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ScheduleCommand
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import org.team9432.lib.Library
import org.team9432.lib.coroutines.LoggedCoroutineRobot
import org.team9432.lib.coroutines.Team8BitRobot.Runtime.*
import org.team9432.lib.simulation.competitionfield.simulations.CompetitionFieldSimulation
import org.team9432.lib.simulation.competitionfield.simulations.Crescendo2024FieldSimulation
import org.team9432.lib.simulation.competitionfield.simulations.IntakeSimulation
import org.team9432.lib.simulation.competitionfield.simulations.SwerveDriveSimulation
import org.team9432.lib.unit.*
import org.team9432.lib.util.afterSimCondition
import org.team9432.lib.util.afterSimDelay
import org.team9432.lib.util.allianceSwitch
import org.team9432.lib.util.applyFlip
import org.team9432.resources.drive.Drive
import org.team9432.resources.drive.DrivetrainConstants.simProfile
import org.team9432.resources.drive.TunerConstants
import org.team9432.resources.drive.gyro.GyroIO
import org.team9432.resources.drive.gyro.GyroIOPigeon2
import org.team9432.resources.drive.gyro.GyroIOSim
import org.team9432.resources.drive.module.ModuleIO
import org.team9432.resources.drive.module.ModuleIOKraken
import org.team9432.resources.drive.module.ModuleIOSim
import org.team9432.resources.flywheels.FlywheelIO
import org.team9432.resources.flywheels.FlywheelIONeo
import org.team9432.resources.flywheels.FlywheelIOSim
import org.team9432.resources.flywheels.Flywheels
import org.team9432.resources.pivot.Pivot
import org.team9432.resources.pivot.PivotIO
import org.team9432.resources.pivot.PivotIONeo
import org.team9432.resources.pivot.PivotIOSim
import org.team9432.resources.rollers.Rollers
import org.team9432.resources.rollers.intake.Intake
import org.team9432.resources.rollers.intake.IntakeIO
import org.team9432.resources.rollers.intake.IntakeIONeo
import org.team9432.resources.rollers.intake.IntakeIOSim
import org.team9432.resources.rollers.loader.Loader
import org.team9432.resources.rollers.loader.LoaderIO
import org.team9432.resources.rollers.loader.LoaderIONeo
import org.team9432.resources.rollers.loader.LoaderIOSim
import org.team9432.vision.Vision
import org.team9432.vision.VisionIO
import org.team9432.vision.VisionIOReal
import org.team9432.vision.VisionIOSim


object Robot: LoggedCoroutineRobot() {
    override val tuningMode = true

    val runtime = if (RobotBase.isReal()) REAL else SIM

    private val controller = CommandXboxController(0)

    private val drive: Drive
    private val flywheels: Flywheels
    private val rollers: Rollers
    private val pivot: Pivot
    private val vision: Vision

    private val fieldSimulation: CompetitionFieldSimulation?
    private val noteSimulation: NoteSimulation?

    init {
        Library.initialize(this, runtime)

        loggerInit()

        when (runtime) {
            REAL -> {
                drive = Drive(
                    GyroIOPigeon2(),
                    ModuleIOKraken(TunerConstants.FrontLeft, TunerConstants.kCANbusName),
                    ModuleIOKraken(TunerConstants.FrontRight, TunerConstants.kCANbusName),
                    ModuleIOKraken(TunerConstants.BackLeft, TunerConstants.kCANbusName),
                    ModuleIOKraken(TunerConstants.BackRight, TunerConstants.kCANbusName)
                )
                flywheels = Flywheels(FlywheelIONeo())
                rollers = Rollers(
                    Intake(IntakeIONeo()),
                    Loader(LoaderIONeo())
                )
                pivot = Pivot(PivotIONeo(0.835.rotations))

                fieldSimulation = null
                noteSimulation = null

                vision = Vision(VisionIOReal())
            }

            SIM -> {
                val gyroIO = GyroIOSim()
                val frontLeft = ModuleIOSim()
                val frontRight = ModuleIOSim()
                val backLeft = ModuleIOSim()
                val backRight = ModuleIOSim()

                drive = Drive(gyroIO, frontLeft, frontRight, backLeft, backRight)
                flywheels = Flywheels(FlywheelIOSim())
                rollers = Rollers(
                    Intake(IntakeIOSim()),
                    Loader(LoaderIOSim())
                )
                pivot = Pivot(PivotIOSim())

                val swerveSim = SwerveDriveSimulation(
                    simProfile,
                    gyroIO,
                    frontLeft,
                    frontRight,
                    backLeft,
                    backRight,
                    startingPose = Pose2d(3.0, 2.0, Rotation2d()).applyFlip(),
                    RobotPosition::resetOdometry
                )

                val intakeWidth = 18.inches
                val intakeX = -14.325.inches - 1.5.inches // The extra 1.5" is because the sim doesn't let notes go under the bumpers

                val intakeSim = IntakeSimulation(
                    startPointOnRobot = Translation2d(intakeX, intakeWidth / 2),
                    endPointOnRobot = Translation2d(intakeX, -intakeWidth / 2),
                    capacity = 1,
                    intakeRunningSupplier = { rollers.isIntaking }
                )

                fieldSimulation = Crescendo2024FieldSimulation(swerveSim).apply {
                    registerIntake(intakeSim)
                    placeGamePiecesOnField()
                }

                noteSimulation = NoteSimulation(
                    robotPoseSupplier = { swerveSim.pose3d.toPose2d() },
                    addGamePieceOnFlyDisplay = { gamePiece -> fieldSimulation.competitionField.addGamePieceOnFly(gamePiece) },
                    intakeSim
                )

                vision = Vision(VisionIOSim(actualRobotPoseSupplier = { swerveSim.objectOnFieldPose2d }))
            }

            REPLAY -> {
                drive = Drive(
                    object: GyroIO {},
                    object: ModuleIO {},
                    object: ModuleIO {},
                    object: ModuleIO {},
                    object: ModuleIO {}
                )
                flywheels = Flywheels(object: FlywheelIO {})
                rollers = Rollers(
                    Intake(object: IntakeIO {}),
                    Loader(object: LoaderIO {})
                )
                pivot = Pivot(object: PivotIO {})

                fieldSimulation = null
                noteSimulation = null

                vision = Vision(object: VisionIO {})
            }
        }

        bindButtons()

        Beambreak

        PortForwarder.add(5800, "10.94.32.11", 5800)
        PortForwarder.add(5800, "10.94.32.12", 5800)
        PortForwarder.add(5800, "photonvision.local", 5800)

        `LEDs!`

        DriverStation.silenceJoystickConnectionWarning(true)
    }


    private fun CommandGenericHID.rumbleCommand() = Commands.startEnd(
        { hid.setRumble(RumbleType.kBothRumble, 1.0) },
        { hid.setRumble(RumbleType.kBothRumble, 0.0) }
    ).asProxy()

    private fun CommandGenericHID.alternatingRumbleCommand(periodSeconds: Double) = Commands.repeatingSequence(
        Commands.runOnce({
            hid.setRumble(RumbleType.kLeftRumble, 1.0)
            hid.setRumble(RumbleType.kRightRumble, 0.0)
        }),
        Commands.waitSeconds(periodSeconds / 2),
        Commands.runOnce({
            hid.setRumble(RumbleType.kLeftRumble, 0.0)
            hid.setRumble(RumbleType.kRightRumble, 1.0)
        }),
        Commands.waitSeconds(periodSeconds / 2),
    ).finallyDo { _ -> hid.setRumble(RumbleType.kBothRumble, 0.0) }.asProxy()

    private fun bindButtons() {
        drive.defaultCommand = drive.run {
            drive.acceptTeleopInput(
                -controller.leftY,
                -controller.leftX,
                controller.leftTriggerAxis - controller.rightTriggerAxis
            )
        }.withName("Teleop Drive")

        fun driveAimCommand(target: () -> Rotation2d) =
            Commands.startEnd({ drive.setAutoAimGoal(target) }, { drive.clearAutoAimGoal() })
                .onlyIf { !RobotState.automationDisabled }

        fun pivotAimCommand(goal: Pivot.Goal) =
            pivot.runGoal(goal).onlyIf { !RobotState.automationDisabled && RobotState.pivotEnabled }

        fun flywheelSpeakerSpeedCommand() =
            Commands.either(
                flywheels.runGoal(Flywheels.Goal.AMP),
                flywheels.runGoal(Flywheels.Goal.SHOOT),
                RobotState::pivotEnabled
            )

        /**** Shooting ****/
        val inSpeakerScoringRange = Trigger(RobotPosition::isInSpeakerScoringRange)
        val inSpeakerPrepareRange = Trigger(RobotPosition::isInSpeakerPrepareRange)

        val readyToShoot = Trigger {
            drive.atAutoAimGoal() &&
                    pivot.atGoal &&
                    flywheels.atGoal
        }.debounce(0.3, Debouncer.DebounceType.kRising)

        // Prepare for a speaker shot
        controller
            .a()
            .and(controller.leftBumper().negate()) // Don't aim and stuff while trying to intake
            .and(inSpeakerPrepareRange.or(RobotState::automationDisabled))
            .whileTrue(
                driveAimCommand { RobotPosition.getStandardAimingParameters().drivetrainAngle }
                    .alongWith(
                        flywheels.runGoal(Flywheels.Goal.SHOOT),
                        pivotAimCommand(Pivot.Goal.SPEAKER_AIM)
                    )
                    .withName("Prepare Speaker")
            )

        // Execute speaker shot
        controller
            .rightBumper()
            .and(controller.a()) // Make sure we are trying to shoot speaker
            .and(readyToShoot)
            .and(inSpeakerScoringRange.or(RobotState::automationDisabled))
            .onTrue(
                Commands.parallel(
                    Commands.waitSeconds(0.5),
                    Commands.waitUntil(controller.rightBumper().negate())
                ).deadlineWith( // Deadline runs the below commands until 0.5 second have passed or the button is released
                    rollers.runGoal(Rollers.Goal.SHOOTER_FEED),
                    flywheels.runGoal(Flywheels.Goal.SHOOT),
                    pivotAimCommand(Pivot.Goal.SPEAKER_AIM),
                    driveAimCommand { RobotPosition.getStandardAimingParameters().drivetrainAngle },
                    Commands.runOnce({ noteSimulation?.animateShoot() })
                )
                    .withName("Shoot Speaker")
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming) // Don't let this be interrupted
                    .finallyDo { _ -> Beambreak.simClear() } // Remove note from the robot in sim
            )

        controller.a().and(inSpeakerScoringRange).and(readyToShoot).whileTrue(controller.alternatingRumbleCommand(0.1))

        // Prepare for an amp shot
        controller
            .b()
            .and(controller.leftBumper().negate()) // Don't aim while trying to intake
            .whileTrue(
                driveAimCommand { 90.degrees.asRotation2d }
                    .alongWith(
                        flywheels.runGoal(Flywheels.Goal.AMP),
                        pivotAimCommand(Pivot.Goal.AMP)
                    )
                    .withName("Prepare Amp")
            )

        // Execute amp shot
        controller
            .rightBumper()
            .and(controller.b()) // Make sure we are trying to shoot amp
            .onTrue(
                Commands.parallel(
                    Commands.waitSeconds(0.5),
                    Commands.waitUntil(controller.rightBumper().negate())
                ).deadlineWith( // Deadline runs the below commands until 0.5 second have passed or the button is released
                    rollers.runGoal(Rollers.Goal.SHOOTER_FEED),
                    flywheels.runGoal(Flywheels.Goal.AMP),
                    pivotAimCommand(Pivot.Goal.AMP),
                    driveAimCommand { 90.degrees.asRotation2d }
                )
                    .withName("Shoot Amp")
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming) // Don't let this be interrupted
                    .finallyDo { _ -> Beambreak.simClear(); noteSimulation?.clearNote() } // Remove note from the robot in sim
            )

        /**** Intake ****/
        controller.leftBumper()
            .and(DriverStation::isEnabled) // I think this makes it so it runs if you are holding the button while it enables
            .whileTrue(
                Commands.parallel(
                    pivotAimCommand(Pivot.Goal.INTAKE),
                    flywheels.runGoal(Flywheels.Goal.NOTE_ALIGN),
                    Commands.waitUntil(pivot::atGoal).andThen(
                        rollers.runGoal(Rollers.Goal.INTAKE)
                            .until(Beambreak::hasNote).afterSimCondition({ noteSimulation!!.hasNote }, { Beambreak.lowerBeambreak.setSimTripped() })
                            .andThen(
                                ScheduleCommand(
                                    Commands.sequence(
                                        Commands.runOnce({ noteSimulation?.animateAlign() }),
                                        rollers.runGoal(Rollers.Goal.INTAKE).until(Beambreak.upperBeambreak::isTripped).afterSimDelay(0.2) { Beambreak.upperBeambreak.setSimTripped() },
                                        rollers.runGoal(Rollers.Goal.ALIGN_FORWARD).until(Beambreak.lowerBeambreak::isClear).afterSimDelay(0.2) { Beambreak.lowerBeambreak.setSimClear() },
                                        rollers.runGoal(Rollers.Goal.ALIGN_REVERSE).until(Beambreak.lowerBeambreak::isTripped).afterSimDelay(0.2) { Beambreak.lowerBeambreak.setSimTripped() },
                                    )
                                        .deadlineWith(flywheels.runGoal(Flywheels.Goal.NOTE_ALIGN))
                                        .alongWith(ScheduleCommand(controller.rumbleCommand().withTimeout(2.0)))
                                        .onlyIf { Beambreak.hasNote }
                                        .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                                        .withTimeout(3.0)
                                        .withName("Note Align")
                                )
                            )

                    )
                ).withName("Teleop Intake")
            )

        controller.start().whileTrue(rollers.runGoal(Rollers.Goal.FLOOR_EJECT).withName("Floor Eject").afterSimDelay(0.5) { Beambreak.simClear(); noteSimulation?.clearNote() })

        /**** Misc. ****/
        controller.back().onTrue(Commands.runOnce({ drive.setGyroAngle(allianceSwitch(blue = Rotation2d(), red = Rotation2d(Math.PI))) }).withName("Gyro Reset"))

        controller.povDown().whileTrue(pivot.runGoal(Pivot.Goal.CUSTOM))
    }

    private fun loggerInit() {
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

        // Command Logging
        val commandCounts: MutableMap<String, Int> = HashMap()
        fun logCommand(command: Command, starting: Boolean) {
            val name = command.name
            val count = commandCounts.getOrDefault(name, 0) + (if (starting) 1 else -1)
            commandCounts[name] = count
            Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), starting)
            Logger.recordOutput("CommandsAll/$name", count > 0)
        }

        CommandScheduler.getInstance().run {
            onCommandInitialize { command -> logCommand(command, true) }
            onCommandFinish { command -> logCommand(command, false) }
            onCommandInterrupt { command -> logCommand(command, false) }
        }

        // Disables protobuf encoding warning, supposedly it's only bad the first few times it's called, we'll have to see
        // Original warning:
        // Warning at org.littletonrobotics.junction.LogTable.put(LogTable.java:429): Logging value to field "/Vision/Results" using protobuf encoding. This may cause high loop overruns, please monitor performance or save the value in a different format. Call "LogTable.disableProtobufWarning()" to disable this message.
        LogTable.disableProtobufWarning()
    }

    override fun periodic() {
        CommandScheduler.getInstance().run()

        fieldSimulation?.updateSimulationWorld()
        fieldSimulation?.competitionField?.updateObjectsToDashboardAndTelemetry()


        // Log CANivore status
        if (runtime == REAL) {
            val canivoreStatus = CANBus.getStatus(TunerConstants.kCANbusName)
            Logger.recordOutput("CANivoreStatus/Status", canivoreStatus.Status.getName())
            Logger.recordOutput("CANivoreStatus/Utilization", canivoreStatus.BusUtilization)
            Logger.recordOutput("CANivoreStatus/OffCount", canivoreStatus.BusOffCount)
            Logger.recordOutput("CANivoreStatus/TxFullCount", canivoreStatus.TxFullCount)
            Logger.recordOutput("CANivoreStatus/ReceiveErrorCount", canivoreStatus.REC)
            Logger.recordOutput("CANivoreStatus/TransmitErrorCount", canivoreStatus.TEC)
        }
    }

    override suspend fun autonomous() {
        Autos(drive, pivot, rollers, flywheels, noteSimulation).farsideTriple().schedule()
//        RobotController.setAction {
//            val trajectoryGroup = Choreo.getTrajectoryGroup("testing")
//            Swerve.resetOdometry(trajectoryGroup.first().getAutoFlippedInitialPose())
//            Swerve.setActualSimPose(trajectoryGroup.first().getAutoFlippedInitialPose())
//            trajectoryGroup.forEachIndexed { index1, it ->
//                println("starting $index1")
//                Logger.recordOutput("Test/TargetEndPose", it.finalPose.applyFlip())
//                Logger.recordOutput("Test/Samples", *it.poses.map { it.applyFlip() }.filterIndexed { index, _ -> index % 4 == 0 }.toTypedArray())
//                Swerve.followChoreo(it)
//                delay(1.seconds)
//            }
//            val selectedAuto = AutoChooser.getAuto()

//            if (selectedAuto == null) {
//                println("[Error] Auto was null")
//                return@setAction
//            }

//            when (selectedAuto) {
//                is FourNote -> RobotFourNote.run(selectedAuto)
//                is FarsideCenterline -> RobotFarsideCenterline.run(selectedAuto)
//                is AmpsideCenterline -> RobotAmpsideCenterline.run(selectedAuto)
//            }
//        }
    }

    override suspend fun disabled() {
//        RobotController.resetRequests()
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