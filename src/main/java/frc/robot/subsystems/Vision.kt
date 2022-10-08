package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants
import java.lang.System.currentTimeMillis
import kotlin.math.atan2

// Should use Drivetrain position for increase accuracy (Drivetrain periodic may not run first which it should)
object Vision : SubsystemBase() {
    private val visionTable = NetworkTableInstance.getDefault().getTable("vision")
    private val isTargetEntry = visionTable.getEntry("is_target")
    private val targetPosEntry = visionTable.getEntry("position")
    private val targetAngleEntry = visionTable.getEntry("rotation")

    var targetPose: Pose2d? = null
        private set

    private fun getMillisSinceUpdate() = if (isTargetEntry.exists()) { currentTimeMillis() - isTargetEntry.lastChange } else { null }

    // Runs before command periodic
    override fun periodic() {
        val lastUpdate = getMillisSinceUpdate()
        targetPose = if (isTargetEntry.getBoolean(false) && lastUpdate != null && lastUpdate <= constants.visionTimeout) {
            val rawPos = targetPosEntry.getDoubleArray(DoubleArray(0))
            val rawRot = targetAngleEntry.getDoubleArray(DoubleArray(0))
            Pose2d(Translation2d(rawPos[0], rawPos[2]), Rotation2d(atan2(rawRot[1], rawRot[0])))
        } else {
            null
        }
    }
}