package frc.robot.commands.test

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

class Drive(private val chassisSpeeds: ChassisSpeeds) : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        Drivetrain.drive(chassisSpeeds)
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
