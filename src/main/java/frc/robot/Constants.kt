package frc.robot

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Translation2d
import kotlin.math.PI

data class SwerveModuleData(val position: Translation2d, val powerMotorID: Int, val angleMotorID: Int, val angleEncoderID: Int)

// (diagonal length / 4) * sqrt of 2
const val HALF_SIDE_LENGTH = (86.6 / 4) * 1.41421356237
val swerveModuleData = mutableListOf(
    SwerveModuleData(Translation2d(HALF_SIDE_LENGTH, HALF_SIDE_LENGTH), 5, 1, 9),
    SwerveModuleData(Translation2d(-HALF_SIDE_LENGTH, -HALF_SIDE_LENGTH), 6, 2, 10),
    SwerveModuleData(Translation2d(-HALF_SIDE_LENGTH, HALF_SIDE_LENGTH), 7, 3, 11),
    SwerveModuleData(Translation2d(HALF_SIDE_LENGTH, -HALF_SIDE_LENGTH), 8, 4, 12))

val swervePowerFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)
val swervePowerPID = PIDController(0.0, 0.0, 0.0)
val swerveAnglePID = PIDController(0.0, 0.0, 0.0)

val autoForwardPID = PIDController(0.0, 0.0, 0.0)
val autoAnglePID = PIDController(0.0, 0.0, 0.0)
const val AUTO_ANGLE_MAX_VEL = 0.0
const val AUTO_ANGLE_MAX_ACC = 0.0

//PI * diameter / (gear ratio * counts per rev)
const val POWER_ENCODER_MULTIPLIER = PI * 0.1016 / (8.14 * 2048)
//PI * diameter / counts per rev
const val ANGLE_ENCODER_MULTIPLIER = 2 * PI / 4096

const val DEADBAND = 0.1
const val ROT_DEADBAND = 0.1

const val DRIVE_SPEED = 1.0
const val ROT_SPEED = 1.0
