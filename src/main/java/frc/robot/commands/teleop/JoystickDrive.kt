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

        // Have rot deadband independent from power deadband
        if (abs(rot) <= constants.rotDeadband && x.pow(2) + y.pow(2) <= constants.powerDeadband * constants.powerDeadband) {
            Drivetrain.stop()
        } else {
            if (fieldOrientated) {
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(x * constants.driveSpeed, y * constants.driveSpeed, rot * constants.rotSpeed, Drivetrain.pose.rotation))
            } else {
                Drivetrain.drive(ChassisSpeeds(x * constants.driveSpeed, y * constants.driveSpeed, rot * constants.rotSpeed))
            }
        }

        Input.setRumble(Drivetrain.getAccelerationSqr() * constants.rumbleFactor)
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}
