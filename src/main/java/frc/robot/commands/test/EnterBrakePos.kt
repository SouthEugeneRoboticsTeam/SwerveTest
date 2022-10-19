package frc.robot.commands.test

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain

class EnterBrakePos : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        Drivetrain.enterBrakePos()
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
