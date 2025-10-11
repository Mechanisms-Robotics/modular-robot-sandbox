// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.PoseEstimator8736;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(Drivetrain subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command driveForwardAuto(Drivetrain drivetrain, PoseEstimator8736 poseEstimator) {
    // Drive forward at a constant speed for 2 seconds, then stop
    return Commands.sequence(
      new InstantCommand(poseEstimator::zeroGyroReversed),
      new RunCommand(
          () -> drivetrain.setDesiredState(new ChassisSpeeds(1.0, 0.0, 0.0)), // 1 m/s forward
          drivetrain).withTimeout(2), // Run for 2 seconds
      new InstantCommand(
          () -> drivetrain.setDesiredState(new ChassisSpeeds(0.0, 0.0, 0.0)), // Stop the robot
          drivetrain));
  }

  public static Command driveForwardPlace(Drivetrain drivetrain, Outtake outtake, PoseEstimator8736 poseEstimator) {
    // Drive forward at a constant speed for 2 seconds, then stop
    return Commands.sequence(
        new InstantCommand(poseEstimator::zeroGyroReversed),
        new RunCommand(
            () -> drivetrain.setDesiredState(new ChassisSpeeds(1.0, 0.0, 0.0)), // 1 m/s forward
            drivetrain).withTimeout(2), // Run for 2 seconds
        new InstantCommand(
            () -> drivetrain.setDesiredState(new ChassisSpeeds(0.0, 0.0, 0.0)), // Stop the robot
            drivetrain),
        outtake.startOuttake(),
        new WaitCommand(5),
        outtake.stopOuttake());
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
