package frc.robot.commands.teleop

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.*
import frc.robot.subsystems.Drivetrain
import java.lang.System.currentTimeMillis
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

class JoystickDrive(private val fieldOrientated: Boolean) : CommandBase() {
    var x = 0.0
    var y = 0.0
    var prevTime = 0L

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        x = 0.0
        y = 0.0

        prevTime = currentTimeMillis()
    }

    override fun execute() {
        var currX = Input.getX()
        var currY = Input.getY()

        if (currX.pow(2) + currY.pow(2) <= constants.powerDeadband * constants.powerDeadband) {
            currX = 0.0
            currY = 0.0
        }

        val currentTime = currentTimeMillis()
        val diffTime = (currentTime - prevTime) / 1000.0
        prevTime = currentTime

        val rateX = (x - currX) / diffTime
        val rateY = (y - currY) / diffTime

        val rateChangeSqr = rateX.pow(2) + rateY.pow(2)
        if (rateChangeSqr <= constants.joystickChangeSpeed * constants.joystickChangeSpeed) {
            x = currX
            y = currY
        } else {
            val rateChange = sqrt(rateChangeSqr)
            x -= rateX * constants.joystickChangeSpeed / rateChange
            y -= rateY * constants.joystickChangeSpeed / rateChange
        }

        var rot = Input.getRot()

        if (abs(rot) <= constants.rotDeadband) {
            rot = 0.0
        }

        if (x.pow(2) + x.pow(2) <= constants.powerDeadband * constants.powerDeadband && abs(rot) <= constants.rotDeadband) {
            Drivetrain.stop()
        } else {
            if (fieldOrientated) {
                Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(x * constants.driveSpeed, y * constants.driveSpeed, rot * constants.rotSpeed, Drivetrain.pose.rotation))
            } else {
                Drivetrain.drive(ChassisSpeeds(x * constants.driveSpeed, y * constants.driveSpeed, rot * constants.rotSpeed))
            }
        }

        // Maybe rumble when driving stops
        Input.setRumble(Drivetrain.getAccelSqr() * constants.rumbleFactor)
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
        Input.setRumble(0.0)
    }
}
