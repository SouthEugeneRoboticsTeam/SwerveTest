package frc.robot.commands.test

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import kotlin.math.PI

class RunTests : SequentialCommandGroup(
    Lock().withTimeout(3.0),
    Drive(ChassisSpeeds(1.0, 0.0, 0.0)).withTimeout(1.0),
    Drive(ChassisSpeeds(0.0, 1.0, 0.0)).withTimeout(1.0),
    Drive(ChassisSpeeds(-1.0, 0.0, 0.0)).withTimeout(1.0),
    Drive(ChassisSpeeds(0.0, -1.0, 0.0)).withTimeout(1.0),
    Drive(ChassisSpeeds(0.0, 0.0, PI)).withTimeout(2.0),
    Drive(ChassisSpeeds(0.0, 0.0, -PI)).withTimeout(2.0),
    Lock())
