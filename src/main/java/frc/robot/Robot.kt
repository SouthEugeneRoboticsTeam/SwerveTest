package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.auto.DriveAprilTag
import frc.robot.commands.teleop.JoystickDrive
import frc.robot.commands.test.RunTests
import frc.robot.subsystems.Drivetrain

object Robot : TimedRobot() {
    private val commandScheduler = CommandScheduler.getInstance()

    private val joystickDrive = JoystickDrive(true)
    private val auto = DriveAprilTag(Translation2d(-1.0, 0.0))
    /*private val auto = DrivePath(TrajectoryGenerator.generateTrajectory(
        Pose2d(0.0, 0.0, Rotation2d(0.0)),
        listOf(Translation2d(1.0, 1.0),
            Translation2d(0.0, 2.0),
            Translation2d(-1.0, 1.0)),
        Pose2d(0.0, 0.0, Rotation2d(0.0)),
        constants.trajectoryConfig), Rotation2d(PI))*/
    private val runTests = RunTests()

    override fun robotPeriodic() {
        Input.update()

        if (constants.tuning) {
            Tuning.update()
        }

        Shuffleboard.update()

        commandScheduler.run()
    }

    override fun teleopInit() {
        joystickDrive.schedule()
    }

    override fun teleopExit() {
        joystickDrive.cancel()
    }

    override fun autonomousInit() {
        Drivetrain.pose = Pose2d()
        auto.schedule()
    }

    override fun autonomousExit() {
        auto.cancel()
    }

    override fun testInit() {
        runTests.schedule()
    }

    override fun testExit() {
        runTests.cancel()
    }

    override fun disabledInit() {
        Input.setRumble(0.0)
    }
}