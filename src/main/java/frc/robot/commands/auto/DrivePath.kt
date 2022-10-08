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

class DrivePath(private val trajectory: Trajectory, private val angle: Rotation2d) : CommandBase(), Reloadable {
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
            ProfiledPIDController(constants.autoAngleP, constants.autoAngleI, constants.autoAngleD,
                TrapezoidProfile.Constraints(constants.autoMaxVel, constants.autoMaxAcc)))
    }

    override fun initialize() {
        startTime = currentTimeMillis()
    }

    override fun execute() {
        Drivetrain.drive(driveController.calculate(Drivetrain.pose, trajectory.sample((currentTimeMillis() - startTime).toDouble() / 1000), angle))
    }

    override fun isFinished(): Boolean {
        return (startTime - currentTimeMillis()) / 1000 > trajectory.totalTimeSeconds
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }

    override fun reload() {
        setController()
    }
}
