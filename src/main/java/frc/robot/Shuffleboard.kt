package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object Shuffleboard {
    private val values = mutableListOf<Pair<String, () -> Double>>()

    init {
        update()
    }

    fun update() {
        for (value in values) {
            SmartDashboard.putNumber(value.first, value.second())
        }
    }
}