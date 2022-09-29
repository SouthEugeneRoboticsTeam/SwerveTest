package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrajectoryConfig
import edu.wpi.first.math.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.commands.auto.DrivePath
import frc.robot.commands.teleop.JoystickDrive
import frc.robot.commands.test.RunTests

object Robot : TimedRobot() {
    private val commandScheduler = CommandScheduler.getInstance()

    private val joystickDrive = JoystickDrive(true)
    private val auto = DrivePath(TrajectoryGenerator.generateTrajectory(
        Pose2d(0.0, 0.0, Rotation2d(0.0)),
        listOf(),
        Pose2d(1.0, 0.0, Rotation2d(0.0)),
        TrajectoryConfig(1.0, 1.0)), Rotation2d(90.0))
    private val runTests = RunTests()

    override fun robotPeriodic() {
        Input.update()
        commandScheduler.run()
    }

    override fun teleopInit() {
        joystickDrive.schedule()
    }

    override fun teleopExit() {
        joystickDrive.cancel()
    }

    override fun autonomousInit() {
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
}