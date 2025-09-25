// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private StructPublisher<Pose3d> posePub =
    NetworkTableInstance.getDefault()
        .getStructTopic("/SimPose", Pose3d.struct)
        .publish();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  public Instant lastSwerveModuleSetTime = Instant.MAX;
  private final static long SWERVE_SET_FREQUECY_SECONDS = 1;

  @Override
  public void disabledPeriodic() {
    
    // ****** TEMPORARY CODE TO TEST DRIVETRAIN
    m_robotContainer.drivetrain.setDesiredState(new ChassisSpeeds(
      0.0, 0.0, 0.0));




    Instant now = Instant.now();
    Duration durationSinceLast = Duration.between(this.lastSwerveModuleSetTime, now);
    if (durationSinceLast.compareTo(Duration.ofSeconds(SWERVE_SET_FREQUECY_SECONDS)) > 0) {
      this.m_robotContainer.setSwerveModulesToEncoders();
      //System.out.println("Zeroed!");
      this.lastSwerveModuleSetTime = now;
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // ****** TEMPORARY CODE TO TEST DRIVETRAIN
    m_robotContainer.drivetrain.setDesiredState(new ChassisSpeeds(
      0.5, 0.0, 0.0));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    posePub.set(new Pose3d(5.0, 2.0, 0.0, new Rotation3d()));
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  //double radiansPerSecond = 0.0;
  public void simulationPeriodic() {
    ChassisSpeeds speeds = new ChassisSpeeds(
      0.0, 0., 1.0);
    this.m_robotContainer.drivetrain.setDesiredState(speeds);
  }
}
