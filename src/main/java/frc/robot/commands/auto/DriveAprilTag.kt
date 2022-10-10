package frc.robot.commands.auto

import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Reloadable
import frc.robot.constants
import frc.robot.subsystems.Drivetrain
import frc.robot.subsystems.Vision
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2

class DriveAprilTag : CommandBase(), Reloadable {
    private var trajectory: Trajectory? = null
    private var angle: Rotation2d? = null

    private lateinit var driveController: HolonomicDriveController
    private var startTime: Long = 0

    init {
        addRequirements(Drivetrain)

        setController()

        registerReload()
    }

    private fun setController() {
        driveController = HolonomicDriveController(
            PIDController(constants.autoForwardP, constants.autoForwardI, constants.autoForwardD),
            PIDController(constants.autoForwardP, constants.autoForwardI, constants.autoForwardD),
            ProfiledPIDController(
                constants.autoAngleP, constants.autoAngleI, constants.autoAngleD,
                TrapezoidProfile.Constraints(constants.autoMaxVel, constants.autoMaxAcc))
        )
    }

    private fun getVisionEndPose(): Pose2d {
        return Pose2d(Drivetrain.pose.translation + Vision.targetPose!!.translation + constants.visionEndOffset, -Vision.targetPose!!.rotation)
    }

    private fun tryCalculatePath() {
        if (Vision.targetPose != null) {
            // Yes it is swerve so the pose rotation shouldn't matter, but it will generate a nicer trajectory than other angles
            trajectory = TrajectoryGenerator.generateTrajectory(Drivetrain.pose, mutableListOf(), getVisionEndPose(), constants.trajectoryConfig)
        }
    }

    private fun shouldRecalculate(): Boolean {
        if (trajectory == null || angle == null) {
            return true
        } else {
            val endPose = trajectory!!.sample(trajectory!!.totalTimeSeconds).poseMeters
            val shouldEndPose = getVisionEndPose()

            // Maybe move to util function
            // No modulo needed because rotation is clamped between -pi and pi
            var rotationDiff = (shouldEndPose.rotation - endPose.rotation).radians
            if (rotationDiff > PI / 2) {
                rotationDiff -= PI
            }

            if (rotationDiff < -PI / 2) {
                rotationDiff += PI
            }

            // Yes could return the line in the if, but this is clearer
            if ((shouldEndPose.translation - endPose.translation).norm >= constants.visionDisRecalc || abs(rotationDiff) >= constants.visionRotRecalc) {
                return true
            }
        }

        return false
    }

    override fun initialize() {
        startTime = System.currentTimeMillis()
    }

    override fun execute() {
        if (shouldRecalculate()) {
            tryCalculatePath()
        }

        if (Vision.targetPose != null) {
            angle = Drivetrain.pose.rotation + Rotation2d(atan2(Vision.targetPose!!.y, Vision.targetPose!!.x))
        }

        if (trajectory != null && angle != null) {
            Drivetrain.drive(driveController.calculate(Drivetrain.pose, trajectory!!.sample((System.currentTimeMillis() - startTime).toDouble() / 1000), angle))
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }

    override fun reload() {
        setController()
    }
}
