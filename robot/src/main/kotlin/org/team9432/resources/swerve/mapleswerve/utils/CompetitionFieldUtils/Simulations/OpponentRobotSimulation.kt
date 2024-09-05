//package org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils.Simulations
//
//import edu.wpi.first.math.geometry.Pose2d
//import edu.wpi.first.math.geometry.Rotation2d
//import edu.wpi.first.math.kinematics.ChassisSpeeds
//import edu.wpi.first.math.util.Units
//import edu.wpi.first.wpilibj.XboxController
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
//import edu.wpi.first.wpilibj2.command.Command
//import edu.wpi.first.wpilibj2.command.CommandScheduler
//import edu.wpi.first.wpilibj2.command.Commands
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
//import org.team9432.resources.swerve.DriveTrainConstants
//import org.team9432.resources.swerve.FieldConstants
//import org.team9432.resources.swerve.HolonomicDriveSubsystem
//import org.team9432.resources.swerve.mapleswerve.utils.MapleJoystickDriveInput
//import java.util.function.Consumer
//
///**
// * simulates an opponent robot on field
// * in physics, the opponent robot behaves just the same as our own robot, it also follows the Holonomic Chassis Physics
// * the difference is, opponent robots are not controlled by the main gamepad
// * it is either controlled by another gamepad to simulate a defense robot
// * or can follow pre-generated paths to simulate opponent robots who are doing cycles
// */
//class OpponentRobotSimulation(private val robotID: Int): HolonomicChassisSimulation(
//    opponentRobotProfile, ROBOT_QUEENING_POSITIONS.get(
//        robotID
//    )
//),
//    HolonomicDriveSubsystem {
////    private val behaviorChooser: SendableChooser<Command> = SendableChooser<Command>()
////    private val disable: Runnable
////    private var speedSetPoint: ChassisSpeeds = ChassisSpeeds()
////    override fun runRawChassisSpeeds(speeds: ChassisSpeeds) {
////        this.speedSetPoint = speeds
////    }
////
////    override fun getPose(): Pose2d {
////        return super.objectOnFieldPose2d
////    }
////
////    override fun setPose(pose: Pose2d) {
////        super.setSimulationWorldPose(pose)
////    }
////
////    override val measuredChassisSpeedsFieldRelative: ChassisSpeeds
////        get() = super<HolonomicChassisSimulation>.measuredChassisSpeedsFieldRelative
////
////    override val chassisMaxLinearVelocityMetersPerSec: Double
////        get() {
////            return profile.robotMaxVelocity
////        }
////    override val chassisMaxAccelerationMetersPerSecSq: Double
////        get() {
////            return profile.robotMaxAcceleration
////        }
////    override val chassisMaxAngularVelocity: Double
////        get() {
////            return profile.maxAngularVelocity
////        }
////    override val chassisMaxAngularAccelerationRadPerSecSq: Double
////        get() {
////            return profile.maxAngularAcceleration
////        }
////
////    override fun updateSimulationSubTick(iterationNum: Int, subPeriodSeconds: Double) {
////        super.simulateChassisBehaviorWithRobotRelativeSpeeds(speedSetPoint)
////    }
////
////    /**
////     * @param id the id of the robot, 0 to 2, this determines where the robot "respawns"
////     */
//////    init {
//////        this.disable = Runnable {
//////            stop()
//////            setSimulationWorldPose(ROBOT_QUEENING_POSITIONS.get(robotID))
//////        }
//////
//////        behaviorChooser.setDefaultOption("Disabled", Commands.runOnce(disable, this))
////////        behaviorChooser.addOption("Auto Cycle", autoCyleRepeadtelyCommand)
//////        val xboxController: XboxController = XboxController(1 + robotID)
//////        behaviorChooser.addOption(
//////            "Joystick Control Left-Handed",
//////            getJoystickDrive(MapleJoystickDriveInput.leftHandedJoystick(xboxController))
//////        )
//////        behaviorChooser.addOption(
//////            "Joystick Control Right-Handed",
//////            getJoystickDrive(MapleJoystickDriveInput.rightHandedJoystick(xboxController))
//////        )
//////        behaviorChooser.onChange(Consumer<Command> { selectedCommand: Command? -> CommandScheduler.getInstance().schedule(selectedCommand) })
//////
//////        SmartDashboard.putData("FieldSimulation/OpponentRobot" + (robotID + 1) + " Behavior", behaviorChooser)
//////    }
////
//////    val autoCyleRepeadtelyCommand: Command
//////        get() {
//////            val cycleForwardPath: PathPlannerPath = MaplePathPlannerLoader.fromPathFile("opponent cycle path " + robotID, constraints)
//////            val cycleBackwardPath: PathPlannerPath = MaplePathPlannerLoader.fromPathFileReversed(
//////                "opponent cycle path " + robotID,
//////                constraints,
//////                GoalEndState(0, cycleForwardPath.getPreviewStartingHolonomicPose().getRotation())
//////            )
//////            val teleportToStartingPose: Command =
//////                Commands.runOnce({ setSimulationWorldPose(cycleBackwardPath.getPreviewStartingHolonomicPose()) }, this)
//////            val cycleForward: Command = OpponentRobotFollowPath(cycleForwardPath, FieldConstants::isSidePresentedAsRed, this)
//////            val cycleBackward: Command = OpponentRobotFollowPath(cycleBackwardPath, FieldConstants::isSidePresentedAsRed, this)
//////
//////            val end: Runnable = Runnable {
//////                stop()
//////                setSimulationWorldPose(ROBOT_QUEENING_POSITIONS.get(robotID))
//////            }
//////
//////            val cycleRepeatedlyAndStop: Command = SequentialCommandGroup(
//////                teleportToStartingPose,
//////                SequentialCommandGroup(
//////                    cycleBackward,
//////                    cycleForward
//////                ).repeatedly()
//////            ).finallyDo(end)
//////            cycleRepeatedlyAndStop.addRequirements(this)
//////            return cycleRepeatedlyAndStop
//////        }
////
//////    fun getJoystickDrive(joystickDriveInput: MapleJoystickDriveInput): Command {
//////        val startingPose: Pose2d = FieldConstants.toCurrentAlliancePose(RED_ROBOTS_STARTING_POSITIONS.get(robotID))
//////        val queeningPose: Pose2d = ROBOT_QUEENING_POSITIONS.get(robotID)
//////        val teleportToStartingPose: Command = Commands.runOnce({ setSimulationWorldPose(startingPose) }, this)
//////        val end: Runnable = Runnable {
//////            setSimulationWorldPose(queeningPose)
//////            stop()
//////        }
//////
//////        return SequentialCommandGroup(
//////            teleportToStartingPose,
//////            Commands.run({ joystickDrivePeriod(joystickDriveInput) }, this)
//////        ).finallyDo(end)
//////    }
////
////    private fun joystickDrivePeriod(driveInput: MapleJoystickDriveInput) {
////        val gamePadSpeeds: ChassisSpeeds = driveInput.getJoystickChassisSpeeds(4.0, 8.0)
////        val gamePadSpeedsInOurDriverStationReference: ChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(gamePadSpeeds, Rotation2d(Math.PI))
////        super.runDriverStationCentricChassisSpeeds(gamePadSpeedsInOurDriverStationReference)
////    }
////
////    /**
////     * a method to test the driving physics
////     * just for testing
////     * in the formal code, we should be using holonomic drive commands
////     */
////    @Deprecated("") fun testDrivingPhysicsWithJoystick(xboxController: XboxController) {
////        val mapleJoystickDriveInput: MapleJoystickDriveInput = MapleJoystickDriveInput.leftHandedJoystick(xboxController)
////        val gamePadSpeeds: ChassisSpeeds = mapleJoystickDriveInput.getJoystickChassisSpeeds(5.0, 10.0)
////        super.runDriverStationCentricChassisSpeeds(gamePadSpeeds)
////    }
////
////    companion object {
////        /* if an opponent robot is not requested to be on field, it queens outside the field for performance */
////        val ROBOT_QUEENING_POSITIONS: Array<Pose2d> = arrayOf<Pose2d>(
////            Pose2d(-6.0, 0.0, Rotation2d()),
////            Pose2d(-4.0, 0.0, Rotation2d()),
////            Pose2d(-2.0, 0.0, Rotation2d())
////        )
////        val RED_ROBOTS_STARTING_POSITIONS: Array<Pose2d> = arrayOf<Pose2d>(
////            Pose2d(15.2, 6.5, Rotation2d()),
////            Pose2d(15.2, 6.0, Rotation2d()),
////            Pose2d(15.2, 5.5, Rotation2d())
////        )
////        val opponentRobotProfile: RobotSimulationProfile = RobotSimulationProfile(
////            4.0,
////            12.0,
////            Math.toRadians(360.0),
////            Units.lbsToKilograms(125.0),
////            DriveTrainConstants.BUMPER_WIDTH_METERS,
////            DriveTrainConstants.BUMPER_LENGTH_METERS,
////            1.0
////        )
////
////        //        private val constraints: PathConstraints = PathConstraints(3.5, 8, Math.toRadians(180.0), Math.toRadians(360.0))
////        private val tolerance: Pose2d = Pose2d(3.0, 3.0, Rotation2d.fromDegrees(20.0))
////    }
//}
