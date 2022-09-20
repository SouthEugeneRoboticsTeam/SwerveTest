package frc.robot

import edu.wpi.first.wpilibj.XboxController

object Input {
    private val controller = XboxController(0)
    private var prevNext = false
    private var currNext = false

    fun update() {
        prevNext = currNext
        currNext = controller.aButton
    }

    fun getY(): Double {
        return controller.rightY
    }

    fun getX(): Double {
        return controller.rightX
    }

    fun getRot(): Double {
        return controller.leftX
    }

    fun getNext(): Boolean {
        return !prevNext && currNext
    }
}