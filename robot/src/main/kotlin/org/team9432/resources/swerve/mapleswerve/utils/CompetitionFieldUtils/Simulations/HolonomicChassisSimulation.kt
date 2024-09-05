package org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils.Simulations

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.dyn4j.dynamics.Body
import org.dyn4j.dynamics.Force
import org.dyn4j.geometry.Geometry
import org.dyn4j.geometry.MassType
import org.dyn4j.geometry.Vector2
import org.team9432.resources.swerve.DriveTrainConstants
import org.team9432.resources.swerve.mapleswerve.utils.CompetitionFieldUtils.Objects.RobotOnFieldDisplay
import org.team9432.resources.swerve.mapleswerve.utils.CustomMaths.GeometryConvertor
import org.team9432.resources.swerve.mapleswerve.utils.CustomMaths.MapleCommonMath
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.withSign

/**
 * simulates the physics behavior of holonomic chassis,
 * with respect to its collision space, friction and motor propelling forces
 */
abstract class HolonomicChassisSimulation(val profile: RobotSimulationProfile, startingPose: Pose2d): Body(), RobotOnFieldDisplay {
    init {
        /* width and height in world reference is flipped */
        val WIDTH_IN_WORLD_REFERENCE: Double = profile.height
        val HEIGHT_IN_WORLD_REFERENCE: Double = profile.width
        super.addFixture(
            Geometry.createRectangle(WIDTH_IN_WORLD_REFERENCE, HEIGHT_IN_WORLD_REFERENCE),
            profile.robotMass / (profile.height * profile.width),
            DriveTrainConstants.BUMPER_COEFFICIENT_OF_FRICTION,
            DriveTrainConstants.BUMPER_COEFFICIENT_OF_RESTITUTION
        )

        super.setMass(MassType.NORMAL)
        super.setLinearDamping(profile.linearVelocityDamping)
        super.setAngularDamping(profile.angularDamping)
        setSimulationWorldPose(startingPose)
    }

    fun setSimulationWorldPose(robotPose: Pose2d) {
        super.transform.set(GeometryConvertor.toDyn4jTransform(robotPose))
        super.linearVelocity.set(0.0, 0.0)
    }

    /**
     * sets the robot's speeds to a given chassis speeds
     * the robot's speeds will jump to the given speeds in a tick
     * this is different from runRawChassisSpeeds(), which applies forces on the chassis and accelerates smoothly according to physics
     */
    protected fun setRobotSpeeds(givenSpeeds: ChassisSpeeds) {
        super.setLinearVelocity(GeometryConvertor.toDyn4jLinearVelocity(givenSpeeds))
        super.setAngularVelocity(givenSpeeds.omegaRadiansPerSecond)
    }

    fun simulateChassisBehaviorWithRobotRelativeSpeeds(desiredChassisSpeedsRobotRelative: ChassisSpeeds) {
        simulateChassisBehaviorWithFieldRelativeSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(desiredChassisSpeedsRobotRelative, objectOnFieldPose2d.getRotation()))
    }

    protected fun simulateChassisBehaviorWithFieldRelativeSpeeds(desiredChassisSpeedsFieldRelative: ChassisSpeeds) {
        super.setAtRest(false)

        val desiredLinearMotionPercent: Vector2 = GeometryConvertor.toDyn4jLinearVelocity(desiredChassisSpeedsFieldRelative)
            .multiply(1.0 / profile.robotMaxVelocity)
        simulateChassisTranslationalBehavior(
            Vector2.create(
                MapleCommonMath.constrainMagnitude(desiredLinearMotionPercent.getMagnitude(), 1.0),
                desiredLinearMotionPercent.getDirection()
            )
        )

        val desiredRotationalMotionPercent: Double = desiredChassisSpeedsFieldRelative.omegaRadiansPerSecond / profile.maxAngularVelocity
        simulateChassisRotationalBehavior(MapleCommonMath.constrainMagnitude(desiredRotationalMotionPercent, 1.0))
    }

    protected fun simulateChassisTranslationalBehavior(desiredLinearMotionPercent: Vector2) {
        val robotRequestedToMoveLinearly: Boolean = desiredLinearMotionPercent.getMagnitude() > 0.03
        if (!robotRequestedToMoveLinearly) {
            simulateTranslationalFrictionNoMotion()
            return
        }
        val forceVec: Vector2 = desiredLinearMotionPercent.copy().multiply(profile.propellingForce)
        super.applyForce(Force(forceVec))
    }

    protected fun simulateTranslationalFrictionNoMotion() {
        val actualLinearPercent: Double = getLinearVelocity().getMagnitude() / profile.robotMaxVelocity
        val robotActuallyMovingLinearly: Boolean = actualLinearPercent > 0.03
        if (robotActuallyMovingLinearly) super.applyForce(
            Force(
                super.linearVelocity.getNormalized().multiply(-profile.frictionForce)
            )
        )
        else super.setLinearVelocity(Vector2())
    }

    protected fun simulateChassisRotationalBehavior(desiredRotationalMotionPercent: Double) {
        val maximumTorque: Double = profile.maxAngularAcceleration * super.getMass().getInertia()
        if (abs(desiredRotationalMotionPercent) > 0.01) {
            super.applyTorque(desiredRotationalMotionPercent * maximumTorque)
            return
        }

        val actualRotationalMotionPercent: Double = abs(getAngularVelocity() / profile.maxAngularVelocity)
        val frictionalTorqueMagnitude: Double = profile.angularFrictionAcceleration * super.getMass().getInertia()
        if (actualRotationalMotionPercent > 0.01) super.applyTorque(frictionalTorqueMagnitude.withSign(-super.getAngularVelocity()))
        else super.setAngularVelocity(0.0)
    }

    override val objectOnFieldPose2d: Pose2d
        get() {
            return GeometryConvertor.toWpilibPose2d(getTransform())
        }

    open val measuredChassisSpeedsRobotRelative: ChassisSpeeds
        get() = ChassisSpeeds.fromFieldRelativeSpeeds(measuredChassisSpeedsFieldRelative, objectOnFieldPose2d.getRotation())

    open val measuredChassisSpeedsFieldRelative: ChassisSpeeds
        get() {
            return GeometryConvertor.toWpilibChassisSpeeds(getLinearVelocity(), getAngularVelocity())
        }

    /**
     * called in every iteration of sub-period
     */
    abstract fun updateSimulationSubTick(iterationNum: Int, subPeriodSeconds: Double)

    class RobotSimulationProfile @JvmOverloads constructor(
        val robotMaxVelocity: Double = DriveTrainConstants.CHASSIS_MAX_VELOCITY,
        val robotMaxAcceleration: Double = DriveTrainConstants.CHASSIS_MAX_ACCELERATION_MPS_SQ,
        val maxAngularVelocity: Double = DriveTrainConstants.CHASSIS_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
        val robotMass: Double = DriveTrainConstants.ROBOT_MASS_KG,
        val width: Double = DriveTrainConstants.BUMPER_WIDTH_METERS,
        val height: Double = DriveTrainConstants.BUMPER_LENGTH_METERS,
        dampingCoefficient: Double = 0.3
    ) {
        val propellingForce: Double

        val frictionForce: Double

        val linearVelocityDamping: Double

        val maxAngularAcceleration: Double

        val angularDamping: Double

        val angularFrictionAcceleration: Double


        init {
            this.propellingForce = robotMaxAcceleration * robotMass
            this.frictionForce = DriveTrainConstants.MAX_FRICTION_ACCELERATION * robotMass
            this.linearVelocityDamping = robotMaxAcceleration / robotMaxVelocity * dampingCoefficient
            this.maxAngularAcceleration = robotMaxAcceleration / (hypot(width, height) / 2)
            this.angularDamping = maxAngularAcceleration / maxAngularVelocity * dampingCoefficient
            this.angularFrictionAcceleration = DriveTrainConstants.CHASSIS_FRICTIONAL_ANGULAR_ACCELERATION
        }

        override fun toString(): String {
            return String.format(
                "RobotProfile { robotMaxVelocity=%.2f, robotMaxAcceleration=%.2f, robotMass=%.2f, " +
                        "propellingForce=%.2f, frictionForce=%.2f, linearVelocityDamping=%.2f, maxAngularVelocity=%.2f, " +
                        "maxAngularAcceleration=%.2f, angularDamping=%.2f, angularFrictionAcceleration=%.2f, width=%.2f, " +
                        "height=%.2f }",
                robotMaxVelocity, robotMaxAcceleration, robotMass, propellingForce, frictionForce, linearVelocityDamping,
                maxAngularVelocity, maxAngularAcceleration, angularDamping, angularFrictionAcceleration, width, height
            )
        }
    }
}
