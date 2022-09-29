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
        val x = Input.getX()
        val y = Input.getY()
        val rot = Input.getRot()

        if (abs(rot) <= ROT_DEADBAND && x.pow(2) + y.pow(2) <= DEADBAND * DEADBAND) {
            Drivetrain.stop()
        } else {
            if (fieldOrientated) {
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(x * DRIVE_SPEED, y * DRIVE_SPEED, rot * ROT_SPEED, Drivetrain.getPose().rotation))
            } else {
                Drivetrain.drive(ChassisSpeeds(x * DRIVE_SPEED, y * DRIVE_SPEED, rot * ROT_SPEED))
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
