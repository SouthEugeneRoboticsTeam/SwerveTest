package frc.robot.commands.auto

import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.*
import frc.robot.subsystems.Drivetrain
import java.lang.System.currentTimeMillis

// Feedforward
class DrivePath(private val trajectory: Trajectory, private val angle: Rotation2d) : CommandBase() {
    private val driveController = HolonomicDriveController(
        PIDController(autoForwardPID.p, autoForwardPID.i, autoForwardPID.d),
        PIDController(autoForwardPID.p, autoForwardPID.i, autoForwardPID.d),
        ProfiledPIDController(autoAnglePID.p, autoAnglePID.i, autoAnglePID.d, TrapezoidProfile.Constraints(AUTO_ANGLE_MAX_VEL, AUTO_ANGLE_MAX_ACC)))
    private var startTime: Long = 0

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        startTime = currentTimeMillis()
    }

    override fun execute() {
        Drivetrain.drive(driveController.calculate(Drivetrain.getPose(), trajectory.sample((currentTimeMillis() - startTime).toDouble() / 1000), angle))
    }

    override fun isFinished(): Boolean {
        return (startTime - currentTimeMillis()) / 1000 > trajectory.totalTimeSeconds
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
