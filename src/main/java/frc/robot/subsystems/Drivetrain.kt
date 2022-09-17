package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.MotorSafety
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.*
import kotlin.math.atan2

class SafetyWrapper(val function: () -> Unit) : MotorSafety() {
    override fun stopMotor() {
        function()
    }

    override fun getDescription(): String {
        return "Dummy"
    }

}

object Drivetrain : SubsystemBase() {
    private val kinematics: SwerveDriveKinematics

    // Instead of lists have list of module classes
    private val powerMotors: Array<TalonFX>
    private val powerPIDControllers: Array<PIDController>
    private val powerFeedforwards: Array<SimpleMotorFeedforward>

    private val angleMotors: Array<CANSparkMax>
    private val anglePIDControllers: Array<PIDController>

    private val centerRotations: Array<Rotation2d>

    private val safetyWrappers: Array<SafetyWrapper>

    private val gyro = AHRS()
    private val moduleStates: Array<SwerveModuleState>
    private val odometry: SwerveDriveOdometry

    init {
        val modulePositions = mutableListOf<Translation2d>()

        val powerMotorsList = mutableListOf<TalonFX>()
        val powerFeedforwardsList = mutableListOf<SimpleMotorFeedforward>()
        val powerPIDControllersList = mutableListOf<PIDController>()

        val angleMotorsList = mutableListOf<CANSparkMax>()
        val anglePIDControllersList = mutableListOf<PIDController>()

        val centerRotationsList = mutableListOf<Rotation2d>()

        val safetyWrappersList = mutableListOf<SafetyWrapper>()

        val moduleStatesList = mutableListOf<SwerveModuleState>()

        for (module in swerveModules) {
            modulePositions.add(module.position)

            val powerMotor = TalonFX(module.powerMotorID)
            powerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
            powerMotor.setNeutralMode(NeutralMode.Brake)
            safetyWrappersList.add(SafetyWrapper{ powerMotor.set(ControlMode.PercentOutput, 0.0) })
            powerMotorsList.add(powerMotor)

            powerPIDControllersList.add(PIDController(swervePowerPID.p, swervePowerPID.i, swervePowerPID.d))
            powerFeedforwardsList.add(SimpleMotorFeedforward(swervePowerFeedforward.ks, swervePowerFeedforward.kv, swervePowerFeedforward.ka))

            val angleMotor = CANSparkMax(module.angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
            angleMotor.idleMode = CANSparkMax.IdleMode.kBrake
            safetyWrappersList.add(SafetyWrapper(angleMotor::stopMotor))
            angleMotorsList.add(angleMotor)

            anglePIDControllersList.add(PIDController(swerveAnglePID.p, swerveAnglePID.i, swerveAnglePID.d))

            centerRotationsList.add(Rotation2d(atan2(module.position.y, module.position.x)))

            moduleStatesList.add(SwerveModuleState())
        }

        kinematics = SwerveDriveKinematics(*modulePositions.toTypedArray())

        powerMotors = powerMotorsList.toTypedArray()
        powerPIDControllers = powerPIDControllersList.toTypedArray()
        powerFeedforwards = powerFeedforwardsList.toTypedArray()

        angleMotors = angleMotorsList.toTypedArray()
        anglePIDControllers = anglePIDControllersList.toTypedArray()

        centerRotations = centerRotationsList.toTypedArray()

        safetyWrappers = safetyWrappersList.toTypedArray()

        moduleStates = moduleStatesList.toTypedArray()
        odometry = SwerveDriveOdometry(kinematics, gyro.rotation2d)
    }

    override fun periodic() {
        for (i in moduleStates.indices) {
            moduleStates[i] = SwerveModuleState(powerMotors[i].selectedSensorVelocity * POWER_ENCODER_MULTIPLIER, Rotation2d(angleMotors[i].encoder.position * ANGLE_ENCODER_MULTIPLIER))
        }

        odometry.update(gyro.rotation2d, *moduleStates)
    }

    fun getPose(): Pose2d {
        return odometry.poseMeters
    }

    private fun feed() {
        for (wrapper in safetyWrappers) {
            wrapper.feed()
        }
    }

    fun drive(chassisSpeeds: ChassisSpeeds) {
        val wantedModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds)

        for (i in wantedModuleStates.indices) {
            val optimized = SwerveModuleState.optimize(wantedModuleStates[i], moduleStates[i].angle)

            powerMotors[i].set(ControlMode.PercentOutput, powerFeedforwards[i].calculate(optimized.speedMetersPerSecond) + powerPIDControllers[i].calculate(moduleStates[i].speedMetersPerSecond, optimized.speedMetersPerSecond))
            angleMotors[i].set(anglePIDControllers[i].calculate(moduleStates[i].angle.radians, optimized.angle.radians))
        }

        feed()
    }

    fun lock() {
        for (powerMotor in powerMotors) {
            powerMotor.set(ControlMode.PercentOutput, 0.0)
        }

        for (i in angleMotors.indices) {
            val angle = Rotation2d(angleMotors[i].encoder.position * ANGLE_ENCODER_MULTIPLIER)
            val optimized = SwerveModuleState.optimize(SwerveModuleState(0.0, centerRotations[i]), angle)

            angleMotors[i].set(anglePIDControllers[i].calculate(angle.radians, optimized.angle.radians))
        }

        feed()
    }

    fun stop() {
        for (powerMotor in powerMotors) {
            powerMotor.set(ControlMode.PercentOutput, 0.0)
        }

        for (angleMotor in angleMotors) {
            angleMotor.stopMotor()
        }

        feed()
    }
}