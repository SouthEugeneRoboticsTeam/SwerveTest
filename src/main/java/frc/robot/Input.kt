package frc.robot

import edu.wpi.first.wpilibj.XboxController

object Input {
    private val controller = XboxController(0)

    fun getY(): Double {
        return controller.rightY
    }

    fun getX(): Double {
        return controller.rightX
    }

    fun getRot(): Double {
        return controller.leftX
    }
}