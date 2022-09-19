package frc.robot

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Translation2d

data class SwerveModuleData(val position: Translation2d, val powerMotorID: Int, val angleMotorID: Int, val angleEncoderID: Int)

val swerveModuleData = mutableListOf(SwerveModuleData(Translation2d(0.0, 0.0), 0, 0, 0))

val swervePowerFeedforward = SimpleMotorFeedforward(0.0, 0.0, 0.0)
val swervePowerPID = PIDController(0.0, 0.0, 0.0)
val swerveAnglePID = PIDController(0.0, 0.0, 0.0)

val autoForwardPID = PIDController(0.0, 0.0, 0.0)
val autoAnglePID = PIDController(0.0, 0.0, 0.0)
const val AUTO_ANGLE_MAX_VEL = 0.0
const val AUTO_ANGLE_MAX_ACC = 0.0

const val POWER_ENCODER_MULTIPLIER = 0.0
const val ANGLE_ENCODER_MULTIPLIER = 0.0

const val DEADBAND = 0.1
const val ROT_DEADBAND = 0.1

const val DRIVE_SPEED = 1.0
const val ROT_SPEED = 1.0
