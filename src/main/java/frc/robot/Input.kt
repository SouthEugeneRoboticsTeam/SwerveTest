package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import frc.robot.subsystems.Drivetrain

object Input {
    private val controller = XboxController(0)
    private var prevNext = false
    private var currNext = false

    fun update() {
        prevNext = currNext
        currNext = controller.aButton

        if (controller.bButton) {
            Drivetrain.setPose(Pose2d(0.0, 0.0, Rotation2d(0.0)))
        }
    }

    fun getY(): Double {
        return controller.leftY
    }

    fun getX(): Double {
        return -controller.leftX
    }

    fun getRot(): Double {
        return controller.rightX
    }

    fun getNext(): Boolean {
        return !prevNext && currNext
    }

    fun setRumble(value: Double) {
        controller.setRumble(GenericHID.RumbleType.kRightRumble, value)
        controller.setRumble(GenericHID.RumbleType.kLeftRumble, value)
    }
}