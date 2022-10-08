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
import frc.robot.Reloadable
import frc.robot.SwerveModuleData
import frc.robot.constants
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.pow

// Rename lock to something more clear
// Limit power and angle change to protect gears and change stop motor as well
class SwerveModule(val powerMotor: TalonFX,
                   private val powerFeedforward: SimpleMotorFeedforward,
                   private val powerPID: PIDController,
                   val angleMotor: CANSparkMax,
                   private val angleEncoder: CANCoder,
                   private val angleOffset: Double,
                   private val anglePID: PIDController,
                   private val centerRotation: Rotation2d,
                   var state: SwerveModuleState) : MotorSafety() {
    init {
        anglePID.enableContinuousInput(-PI, PI)
    }

    private fun getVelocity(): Double {
        return powerMotor.selectedSensorVelocity * constants.powerEncoderMultiplier
    }

    private fun getAngle(): Rotation2d {
        return Rotation2d(angleEncoder.absolutePosition * constants.angleEncoderMultiplier - angleOffset)
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

object Drivetrain : SubsystemBase(), Reloadable {
    private val kinematics: SwerveDriveKinematics
    private val imu = AHRS()
    private val odometry: SwerveDriveOdometry

    private var modules: Array<SwerveModule>

    init {
        val modulePositions = mutableListOf<Translation2d>()

        val modulesList = mutableListOf<SwerveModule>()

        for (moduleData in constants.swerveModuleData) {
            val powerMotor = TalonFX(moduleData.powerMotorID)
            powerMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
            powerMotor.setNeutralMode(NeutralMode.Brake)

            val angleMotor = CANSparkMax(moduleData.angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless)
            angleMotor.idleMode = CANSparkMax.IdleMode.kBrake
            angleMotor.inverted = true

            modulesList.add(createModule(powerMotor, angleMotor, moduleData))
        }

        modules = modulesList.toTypedArray()

        kinematics = SwerveDriveKinematics(*modulePositions.toTypedArray())
        odometry = SwerveDriveOdometry(kinematics, imu.rotation2d)

        registerReload()
    }

    private fun createModule(powerMotor: TalonFX, angleMotor: CANSparkMax, moduleData: SwerveModuleData): SwerveModule {
        return SwerveModule(powerMotor,
            SimpleMotorFeedforward(constants.swervePowerS, constants.swervePowerV, constants.swervePowerA),
            PIDController(constants.swervePowerP, constants.swervePowerI, constants.swervePowerD),
            angleMotor,
            CANCoder(moduleData.angleEncoderID),
            moduleData.angleOffset,
            PIDController(constants.swerveAngleP, constants.swerveAngleI, constants.swerveAngleD),
            Rotation2d(atan2(moduleData.position.y, moduleData.position.x)),
            SwerveModuleState())
    }

    override fun reload() {
        val modulesList = mutableListOf<SwerveModule>()

        for (i in constants.swerveModuleData.indices) {
            val moduleData = constants.swerveModuleData[i]
            val module = modules[i]

            modulesList.add(createModule(module.powerMotor, module.angleMotor, moduleData))
        }

        modules = modulesList.toTypedArray()
    }

    override fun periodic() {
        val states = mutableListOf<SwerveModuleState>()

        for (module in modules) {
            module.updateState()
            states.add(module.state)
        }

        odometry.update(imu.rotation2d, *states.toTypedArray())
    }

    fun getPose(): Pose2d {
        return odometry.poseMeters
    }

    fun setPose(pose: Pose2d) {
        return odometry.resetPosition(pose, imu.rotation2d)
    }

    fun getAccelerationSqr(): Double {
        return (imu.worldLinearAccelX.pow(2) + imu.worldLinearAccelY.pow(2) + imu.worldLinearAccelZ.pow(2)).toDouble()
    }

    private fun feed() {
        for (module in modules) {
            module.feed()
        }
    }

    fun drive(chassisSpeeds: ChassisSpeeds) {
        // Maybe desaturate wheel speeds
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