package org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils.Simulations

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import org.dyn4j.geometry.Vector2
import org.team9432.Robot
import org.team9432.resources.swerve.DriveTrainConstants.CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC
import org.team9432.resources.swerve.DriveTrainConstants.DRIVE_INERTIA
import org.team9432.resources.swerve.DriveTrainConstants.DRIVE_KINEMATICS
import org.team9432.resources.swerve.DriveTrainConstants.MAX_FRICTION_ACCELERATION
import org.team9432.resources.swerve.DriveTrainConstants.MAX_FRICTION_FORCE_PER_MODULE
import org.team9432.resources.swerve.DriveTrainConstants.MODULE_TRANSLATIONS
import org.team9432.resources.swerve.DriveTrainConstants.ROBOT_MASS_KG
import org.team9432.resources.swerve.DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD
import org.team9432.resources.swerve.DriveTrainConstants.WHEEL_RADIUS_METERS
import org.team9432.resources.swerve.OdometryThread
import org.team9432.resources.swerve.gyro.GyroIOSim
import org.team9432.resources.swerve.mapleswerve.MapleTimeUtils
import org.team9432.resources.swerve.mapleswerve.utils.CustomMaths.GeometryConvertor
import org.team9432.resources.swerve.module.ModuleIOSim
import java.util.function.Consumer
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.min

/**
 * simulates the behavior of our robot
 * it has all the physics behavior as a simulated holonomic chassis
 * in addition to that, it simulates the swerve module behaviors
 * the class is like the bridge between ModuleIOSim and HolonomicChassisSimulation
 * it reads the motor power from ModuleIOSim
 * and feed the result of the physics simulation back to ModuleIOSim, to simulate the odometry encoders' readings
 */
class SwerveDriveSimulation(
    gyroIOSim: GyroIOSim,
    frontLeft: ModuleIOSim, frontRight: ModuleIOSim, backLeft: ModuleIOSim, backRight: ModuleIOSim,
    startingPose: Pose2d,
    resetOdometryCallBack: Consumer<Pose2d>,
): HolonomicChassisSimulation(RobotSimulationProfile(), startingPose) {
    private val gyroIOSim: GyroIOSim
    private val modules: Array<ModuleIOSim>
    private val resetOdometryCallBack: Consumer<Pose2d>

    init {
        this.gyroIOSim = gyroIOSim
        this.modules = arrayOf<ModuleIOSim>(frontLeft, frontRight, backLeft, backRight)
        this.resetOdometryCallBack = resetOdometryCallBack
        resetOdometryToActualRobotPose()
        println("swerve drive sim profile: " + RobotSimulationProfile())
    }

    fun resetOdometryToActualRobotPose() {
        resetOdometryCallBack.accept(objectOnFieldPose2d)
    }

    override fun updateSimulationSubTick(tickNum: Int, tickSeconds: Double) {
        for (i in modules.indices) moduleSimulationSubTick(
            objectOnFieldPose2d,
            modules.get(i),
            MODULE_TRANSLATIONS.get(i),
            tickNum, tickSeconds
        )

        simulateFrictionForce()

        gyroSimulationSubTick(
            super.objectOnFieldPose2d.getRotation(),
            super.getAngularVelocity(),
            tickNum
        )
    }

    private fun moduleSimulationSubTick(
        robotWorldPose: Pose2d,
        module: ModuleIOSim,
        moduleTranslationOnRobot: Translation2d,
        tickNum: Int,
        tickPeriodSeconds: Double,
    ) {
        /* update the DC motor simulation of the steer */
        module.updateSim(tickPeriodSeconds)

        /* simulate the propelling force of the module */
        val moduleWorldFacing: Rotation2d = module.simulationSteerFacing.plus(robotWorldPose.rotation)
        val moduleWorldPosition: Vector2 = GeometryConvertor.toDyn4jVector2(
            robotWorldPose.translation
                .plus(moduleTranslationOnRobot.rotateBy(robotWorldPose.rotation))
        )
        var actualPropellingForceOnFloorNewtons: Double = module.simulationTorque / WHEEL_RADIUS_METERS
        val skidding: Boolean = abs(actualPropellingForceOnFloorNewtons) > MAX_FRICTION_FORCE_PER_MODULE
        if (skidding) actualPropellingForceOnFloorNewtons = Math.copySign(MAX_FRICTION_FORCE_PER_MODULE, actualPropellingForceOnFloorNewtons)
        super.applyForce(
            Vector2.create(actualPropellingForceOnFloorNewtons, moduleWorldFacing.radians),
            moduleWorldPosition
        )


        val floorVelocity: Vector2 = super.getLinearVelocity(moduleWorldPosition)
        val floorVelocityProjectionOnWheelDirectionMPS: Double = floorVelocity.magnitude * cos(floorVelocity.getAngleBetween(Vector2(moduleWorldFacing.radians)))

        if (skidding)  /* if the chassis is skidding, the toque will cause the wheels to spin freely */
            module.physicsSimulationResults.driveWheelFinalVelocityRadPerSec += module.simulationTorque / DRIVE_INERTIA * tickPeriodSeconds
        else  /* otherwise, the floor velocity is projected to the wheel */
            module.physicsSimulationResults.driveWheelFinalVelocityRadPerSec = floorVelocityProjectionOnWheelDirectionMPS / WHEEL_RADIUS_METERS

        module.physicsSimulationResults.odometrySteerPositions[tickNum] = module.simulationSteerFacing
        module.physicsSimulationResults.driveWheelFinalRevolutions += Units.radiansToRotations(
            module.physicsSimulationResults.driveWheelFinalVelocityRadPerSec * tickPeriodSeconds
        )
        module.physicsSimulationResults.odometryDriveWheelRevolutions[tickNum] = module.physicsSimulationResults.driveWheelFinalRevolutions
    }

    private fun simulateFrictionForce() {
        val speedsDifference: ChassisSpeeds = differenceBetweenFloorAndFreeSpeed
        val translationalSpeedsDifference: Translation2d = Translation2d(speedsDifference.vxMetersPerSecond, speedsDifference.vyMetersPerSecond)
        val forceMultiplier: Double = min(translationalSpeedsDifference.norm * 3, 1.0)
        super.applyForce(
            Vector2.create(
                forceMultiplier * MAX_FRICTION_ACCELERATION * ROBOT_MASS_KG,
                translationalSpeedsDifference.angle.radians
            )
        )

        if (abs(desiredSpeedsFieldRelative.omegaRadiansPerSecond) / CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC
            < 0.01) simulateChassisRotationalBehavior(0.0)
    }

    private val differenceBetweenFloorAndFreeSpeed: ChassisSpeeds
        get() {
            var chassisFreeSpeedsFieldRelative: ChassisSpeeds = freeSpeedsFieldRelative

            val freeSpeedMagnitude: Double = hypot(chassisFreeSpeedsFieldRelative.vxMetersPerSecond, chassisFreeSpeedsFieldRelative.vyMetersPerSecond)
            val floorSpeedMagnitude: Double = hypot(measuredChassisSpeedsFieldRelative.vxMetersPerSecond, measuredChassisSpeedsFieldRelative.vyMetersPerSecond)
            if (freeSpeedMagnitude > floorSpeedMagnitude) chassisFreeSpeedsFieldRelative = chassisFreeSpeedsFieldRelative.times(floorSpeedMagnitude / freeSpeedMagnitude)

            return chassisFreeSpeedsFieldRelative.minus(measuredChassisSpeedsFieldRelative)
        }

    private val freeSpeedsFieldRelative: ChassisSpeeds
        get() = ChassisSpeeds.fromRobotRelativeSpeeds(
            DRIVE_KINEMATICS.toChassisSpeeds(*modules.map(ModuleIOSim::simulationSwerveState).toTypedArray()),
            objectOnFieldPose2d.getRotation()
        )

    private val desiredSpeedsFieldRelative: ChassisSpeeds
        get() {
            return ChassisSpeeds.fromRobotRelativeSpeeds(
                DRIVE_KINEMATICS.toChassisSpeeds(*modules.map(ModuleIOSim::desiredSwerveState).toTypedArray()),
                objectOnFieldPose2d.getRotation()
            )
        }

    private fun gyroSimulationSubTick(
        currentFacing: Rotation2d,
        angularVelocityRadPerSec: Double,
        tickNum: Int,
    ) {
        val results: GyroIOSim.GyroPhysicsSimulationResults = gyroIOSim.gyroPhysicsSimulationResults
        results.robotAngularVelocityRadPerSec = angularVelocityRadPerSec
        results.odometryYawPositions[tickNum] = currentFacing
        results.hasReading = true
    }

    class OdometryThreadSim: OdometryThread {
        override fun updateInputs(inputs: OdometryThread.OdometryThreadInputs) {
            inputs.measurementTimeStamps = DoubleArray(SIMULATION_TICKS_IN_1_PERIOD)
            val robotStartingTimeStamps: Double = MapleTimeUtils.logTimeSeconds
            val iterationPeriodSeconds: Double = Robot.period / SIMULATION_TICKS_IN_1_PERIOD
            for (i in 0 until SIMULATION_TICKS_IN_1_PERIOD) inputs.measurementTimeStamps[i] = robotStartingTimeStamps + i * iterationPeriodSeconds
        }
    }
}