package frc.robot.commands.test

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

class Lock : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        Drivetrain.lock()
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
