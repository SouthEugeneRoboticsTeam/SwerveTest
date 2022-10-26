package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.Drivetrain

object Shuffleboard {
    private val field = Field2d()
    private val values = mutableListOf<Pair<String, () -> Double>>()

    init {
        update()

        SmartDashboard.putData(field)
    }

    fun update() {
        field.robotPose = Drivetrain.pose

        for (value in values) {
            SmartDashboard.putNumber(value.first, value.second())
        }
    }
}