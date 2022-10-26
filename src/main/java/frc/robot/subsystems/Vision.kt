package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.lang.System.currentTimeMillis
import kotlin.math.atan2

object Vision : SubsystemBase() {
    private val visionTable = NetworkTableInstance.getDefault().getTable("Vision")
    private val isTargetEntry = visionTable.getEntry("Is Target")
    private val targetPosEntry = visionTable.getEntry("Position")
    private val targetAngleEntry = visionTable.getEntry("Rotation")

    var targetPose: Pose2d? = null
        private set

    private fun getMillisSinceUpdate() = if (isTargetEntry.exists()) { currentTimeMillis() - isTargetEntry.lastChange } else { null }

    // Runs before command periodic
    override fun periodic() {
        val lastUpdate = getMillisSinceUpdate()
        targetPose = if (isTargetEntry.getBoolean(false)) {// && lastUpdate != null && lastUpdate <= constants.visionTimeout) {
            val rawPos = targetPosEntry.getDoubleArray(DoubleArray(0))
            val rawRot = targetAngleEntry.getDoubleArray(DoubleArray(0))
            // Drivetrain.pose.translation may be from the previous periodic
            Pose2d(Translation2d(rawPos[0], rawPos[2]).rotateBy(Drivetrain.pose.rotation) + Drivetrain.pose.translation, Rotation2d(atan2(rawRot[1], rawRot[0])))
        } else {
            null
        }
    }
}