// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  Translation2d frontLeftModuleLocation = new Translation2d(1.0, 1.0);
  Translation2d frontRightModuleLocation = new Translation2d(1.0, -1.0);
  Translation2d backLeftModuleLocation = new Translation2d(-1.0, 1.0);
  Translation2d backRightModuleLocation = new Translation2d(-1.0, -1.0);

  public final Drivetrain drivetrain = new Drivetrain(
    frontLeftModuleLocation, frontRightModuleLocation, backLeftModuleLocation, backRightModuleLocation
  );

  private static final int CONTROLLER_PORT = 0;
  private final CommandPS4Controller controller = new CommandPS4Controller(CONTROLLER_PORT);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  public void setSwerveModulesToEncoders() {
    this.drivetrain.setModulesToEncoders();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
      new RunCommand(
          () -> {
              double forward = -controller.getLeftY(); // Negative to match FRC convention
              double strafe = controller.getLeftX();
              double rotation = controller.getRightX();

              // Apply deadbands and scaling
              final double DEADBAND = 0.04;
              forward = Math.abs(forward) > DEADBAND ? forward : 0.0;
              strafe = Math.abs(strafe) > DEADBAND ? strafe : 0.0;
              rotation = Math.abs(rotation) > DEADBAND ? rotation : 0.0;

              // Scale to max speed
              double MAX_SPEED_METERS_PER_SEC = 1.0; // Set your max speed
              double MAX_ANGULAR_RAD_PER_SEC = Math.PI; // Set your max rotation speed

              ChassisSpeeds speeds = new ChassisSpeeds(
                  forward * MAX_SPEED_METERS_PER_SEC,
                  strafe * MAX_SPEED_METERS_PER_SEC,
                  rotation * MAX_ANGULAR_RAD_PER_SEC
              );

              // Pass to your swerve subsystem
              drivetrain.setDesiredState(speeds);
          },
          drivetrain
      )
  );
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(drivetrain);
  }
}
