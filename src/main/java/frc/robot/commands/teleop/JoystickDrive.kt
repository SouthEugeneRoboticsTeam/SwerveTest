package frc.robot.commands.teleop

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.*
import frc.robot.subsystems.Drivetrain
import kotlin.math.abs
import kotlin.math.pow

class JoystickDrive(private val fieldOrientated: Boolean) : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        var x = Input.getX()
        var y = Input.getY()
        var rot = Input.getRot()

        if (x.pow(2) + y.pow(2) <= constants.powerDeadband * constants.powerDeadband) {
            x = 0.0
            y = 0.0
        }

        if (abs(rot) <= constants.rotDeadband) {
            rot = 0.0
        }

        if (fieldOrientated) {
            Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(x * constants.driveSpeed, y * constants.driveSpeed, rot * constants.rotSpeed, Drivetrain.pose.rotation))
        } else {
            Drivetrain.drive(ChassisSpeeds(x * constants.driveSpeed, y * constants.driveSpeed, rot * constants.rotSpeed))
        }

        // Maybe rumble when driving stops
        Input.setRumble(Drivetrain.getAccel() * constants.rumbleFactor)
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
        Input.setRumble(0.0)
    }
}
