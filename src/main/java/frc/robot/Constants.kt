package frc.robot

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrajectoryConfig
import kotlin.math.PI

data class SwerveModuleData(val position: Translation2d, val powerMotorID: Int, val angleMotorID: Int, val angleEncoderID: Int, val angleOffset: Double)

object TunableConstants {
    val swervePowerS = TunableNumber("Swerve Power S", 0.0)
    val swervePowerV = TunableNumber("Swerve Power V", 0.0)
    val swervePowerA = TunableNumber("Swerve Power A", 0.0)

    val swervePowerP = TunableNumber("Swerve Power P", 0.3)
    val swervePowerI = TunableNumber("Swerve Power I", 0.0)
    val swervePowerD = TunableNumber("Swerve Power D", 0.0)

    val swerveAngleP = TunableNumber("Swerve Angle P", 0.2)
    val swerveAngleI = TunableNumber("Swerve Angle I", 0.0)
    val swerveAngleD = TunableNumber("Swerve Angle D", 0.0)

    val autoForwardP = TunableNumber("Auto Forward P", 0.0)
    val autoForwardI = TunableNumber("Auto Forward I", 0.0)
    val autoForwardD = TunableNumber("Auto Forward D", 0.0)

    val autoAngleP = TunableNumber("Auto Angle P", 0.0)
    val autoAngleI = TunableNumber("Auto Angle I", 0.0)
    val autoAngleD = TunableNumber("Auto Angle D", 0.0)

    val autoMaxVel = TunableNumber("Auto Max Vel", 0.0)
    val autoMaxAcc = TunableNumber("Auto Max Acc", 0.0)
}

// Maybe separate into true constants and tunable constants to make clear what needs to be reloaded
class Constants {
    // (diagonal length / 4) * sqrt of 2
    private val halfSideLength = (0.866 / 4) * 1.41421356237
    val swerveModuleData = mutableListOf(
        SwerveModuleData(Translation2d(halfSideLength, -halfSideLength), 5, 2, 10, 5.26 - (PI / 2)),
        SwerveModuleData(Translation2d(-halfSideLength, -halfSideLength), 6, 3, 11, 0.29 + (PI / 2)),
        SwerveModuleData(Translation2d(halfSideLength, halfSideLength), 7, 1, 12, 4.77 + (PI / 2)),
        SwerveModuleData(Translation2d(-halfSideLength, halfSideLength), 8, 4, 9, 4.76 + (PI / 2))
    )

    val swervePowerS = TunableConstants.swervePowerS.value
    val swervePowerV = TunableConstants.swervePowerV.value
    val swervePowerA = TunableConstants.swervePowerA.value

    val swervePowerP = TunableConstants.swervePowerP.value
    val swervePowerI = TunableConstants.swervePowerI.value
    val swervePowerD = TunableConstants.swervePowerD.value

    val swerveAngleP = TunableConstants.swerveAngleP.value
    val swerveAngleI = TunableConstants.swerveAngleI.value
    val swerveAngleD = TunableConstants.swerveAngleD.value

    val autoForwardP = TunableConstants.autoForwardP.value
    val autoForwardI = TunableConstants.autoForwardI.value
    val autoForwardD = TunableConstants.autoForwardD.value

    val autoAngleP = TunableConstants.autoAngleP.value
    val autoAngleI = TunableConstants.autoAngleI.value
    val autoAngleD = TunableConstants.autoAngleD.value

    val autoMaxVel = TunableConstants.autoMaxVel.value
    val autoMaxAcc = TunableConstants.autoMaxAcc.value

    // PI * diameter / (gear ratio * counts per rev)
    val powerEncoderMultiplier = PI * 0.1016 / (8.14 * 2048)

    // Degrees to radians
    val angleEncoderMultiplier = 0.01745329251

    val powerDeadband = 0.1
    val rotDeadband = 0.1

    val driveSpeed = 1.0
    val rotSpeed = 1.0

    val rumbleFactor = 0.1

    val visionTimeout = 200
    val visionEndOffset = Translation2d(0.0, 1.0)
    val visionDisRecalc = 0.2
    val visionRotRecalc = 0.3

    val trajectoryConfig = TrajectoryConfig(1.0, 1.0)

    val tuning = false
}

var constants = Constants()