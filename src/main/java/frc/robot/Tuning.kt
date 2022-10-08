package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.lang.ref.WeakReference

interface Reloadable {
    fun registerReload() {
        Tuning.reloadRefs.add(WeakReference(this))
    }

    fun reload()
}

class TunableNumber(private val name: String, var value: Double) {
    init {
        Tuning.tunableNumbers.add(this)

        SmartDashboard.putNumber(name, value)
    }

    fun update(): Boolean {
        val curr = SmartDashboard.getNumber(name, value)
        if (value != curr) {
            value = curr
            return true
        }

        return false
    }
}

object Tuning {
    val reloadRefs = mutableListOf<WeakReference<Reloadable>>()
    val tunableNumbers = mutableListOf<TunableNumber>()

    fun update() {
        var change = false
        for (tunableNumber in tunableNumbers) {
            if (tunableNumber.update()) {
                change = true
            }
        }

        if (change) {
            constants = Constants()

            reloadRefs.removeIf { it.get() == null }

            for (ref in reloadRefs) {
                // Would use !! but the garbage collector could run between clean and this
                ref.get()?.reload()
            }
        }
    }
}
