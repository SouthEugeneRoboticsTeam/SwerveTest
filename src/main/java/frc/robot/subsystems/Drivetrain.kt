package frc.robot.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.TalonFX
import com.ctre.phoenix.sensors.CANCoder
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
import kotlin.math.PI
import kotlin.math.atan2

// Rename lock to something more clear
class SwerveModule(private val powerMotor: TalonFX,
                   private val powerFeedforward: SimpleMotorFeedforward,
                   private val powerPID: PIDController,
                   private val angleMotor: CANSparkMax,
                   private val angleEncoder: CANCoder,
                   private val angleOffset: Double,
                   private val anglePID: PIDController,
                   private val centerRotation: Rotation2d,
                   var state: SwerveModuleState) : MotorSafety() {
    init {
        anglePID.enableContinuousInput(-PI, PI)
    }

    private fun getVelocity(): Double {
        return powerMotor.selectedSensorVelocity * POWER_ENCODER_MULTIPLIER
    }

    private fun getAngle(): Rotation2d {
        return Rotation2d(angleEncoder.absolutePosition * ANGLE_ENCODER_MULTIPLIER - angleOffset)
    }

    // Should be called in periodic
    fun updateState() {
        state = SwerveModuleState(getVelocity(), getAngle())
    }

    fun set(wanted: SwerveModuleState) {
        // Using state because it should be updated and getVelocity and getAngle (probably) spend time over CAN
        val optimized = SwerveModuleState.optimize(wanted, state.angle)

        powerMotor.set(ControlMode.PercentOutput, powerFeedforward.calculate(optimized.speedMetersPerSecond) + powerPID.calculate(state.speedMetersPerSecond, optimized.speedMetersPerSecond))
        angleMotor.set(anglePID.calculate(state.angle.radians, optimized.angle.radians))
    }

    fun lock() {
        set(SwerveModuleState(0.0, centerRotation))
    }

    override fun stopMotor() {
        powerMotor.set(ControlMode.PercentOutput, 0.0)
        angleMotor.stopMotor()
    }

    override fun getDescription(): String {
        return "Swerve Module"
    }
}

object Drivetrain : SubsystemBase() {
    private val kinematics: SwerveDriveKinematics
    private val gyro = AHRS()
    private val odometry: SwerveDriveOdometry

    private val modules: Array<SwerveModule>

    init {
        val modulePositions = mutableListOf<Translation2d>()

        val modulesList = mutableListOf<SwerveModule>()

        for (moduleData in swerveModuleData) {
            modulePositions.add(moduleData.position)

            val powerMotor = TalonFX(moduleData.powerMotorID)
            powerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
            powerMotor.setNeutralMode(NeutralMode.Brake)

            val angleMotor = CANSparkMax(moduleData.angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
            angleMotor.idleMode = CANSparkMax.IdleMode.kBrake

            modulesList.add(SwerveModule(powerMotor,
                SimpleMotorFeedforward(swervePowerFeedforward.ks, swervePowerFeedforward.kv, swervePowerFeedforward.ka),
                PIDController(swervePowerPID.p, swervePowerPID.i, swervePowerPID.d),
                angleMotor,
                CANCoder(moduleData.angleEncoderID),
                moduleData.angleOffset,
                PIDController(swerveAnglePID.p, swerveAnglePID.i, swerveAnglePID.d),
                Rotation2d(atan2(moduleData.position.y, moduleData.position.x)),
                SwerveModuleState()))
        }

        kinematics = SwerveDriveKinematics(*modulePositions.toTypedArray())
        odometry = SwerveDriveOdometry(kinematics, gyro.rotation2d)

        modules = modulesList.toTypedArray()
    }

    override fun periodic() {
        val states = mutableListOf<SwerveModuleState>()

        for (module in modules) {
            module.updateState()
            states.add(module.state)
        }

        odometry.update(gyro.rotation2d, *states.toTypedArray())
    }

    fun getPose(): Pose2d {
        return odometry.poseMeters
    }

    fun setPose(pose: Pose2d) {
        return odometry.resetPosition(pose, gyro.rotation2d)
    }

    private fun feed() {
        for (module in modules) {
            module.feed()
        }
    }

    fun drive(chassisSpeeds: ChassisSpeeds) {
        val wantedStates = kinematics.toSwerveModuleStates(chassisSpeeds)

        for (i in wantedStates.indices) {
            modules[i].set(wantedStates[i])
        }

        feed()
    }

    fun lock() {
        for (module in modules) {
            module.lock()
        }

        feed()
    }

    fun stop() {
        for (module in modules) {
            module.stopMotor()
        }

        feed()
    }
}