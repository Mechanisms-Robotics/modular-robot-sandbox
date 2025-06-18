package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  SwerveDriveKinematics kinematics;
  ChassisSpeeds desiredChassisSpeeds;

  public Drivetrain(
    Translation2d frontLeftModuleLocation,
    Translation2d frontRightModuleLocation,
    Translation2d backLeftModuleLocation,
    Translation2d backRightModuleLocation) {

      this.kinematics = new SwerveDriveKinematics(
        frontLeftModuleLocation,
        frontRightModuleLocation,
        backLeftModuleLocation,
        backRightModuleLocation);
        
      }
      
      public void setDesiredState(ChassisSpeeds desiredChassisSpeeds) {
        this.desiredChassisSpeeds = desiredChassisSpeeds;
      }
      
      public ChassisSpeeds getDesiredState() {
        return this.desiredChassisSpeeds;
      }
      
      @Override
      public void periodic() {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desiredChassisSpeeds);
        // TODO Working here
      }








  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
