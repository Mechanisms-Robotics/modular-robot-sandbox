package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private static final boolean OUTPUT_TO_SMART_DASH = true;

    private static final double WHEEL_RADIUS_METERS = 0.0508; // from the internet
    private static final double STEERING_GEAR_RATIO = 150.0/7.0; // from SDS website
    private static final double DRIVE_GEAR_RATIO = 6.75; // L2 from SDS website

    private final int steeringMotorCANId;

    private final TalonFX steeringMotor;
    private final TalonFX driveMotor;

    private final Canandmag encoder;
  
    public SwerveModule(int steeringMotorCANId, int driveMotorCANId, int encoderCANId) {
        this.steeringMotorCANId = steeringMotorCANId;

        this.steeringMotor = new TalonFX(steeringMotorCANId);
        this.driveMotor = new TalonFX(driveMotorCANId);
        this.encoder = new Canandmag(encoderCANId);

        var steeringConfigs = new TalonFXConfiguration();
        steeringConfigs.Slot0.kP = 0.04;
        steeringConfigs.Slot0.kI = 0.0;
        steeringConfigs.Slot0.kD = 0.0;
        steeringConfigs.Slot0.kV = 0.0; // feedforward term
        this.steeringMotor.getConfigurator().apply(steeringConfigs);

        var driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kP = 0.02;
        driveConfigs.Slot0.kI = 0.0;
        driveConfigs.Slot0.kD = 0.0;
        driveConfigs.Slot0.kV = 0.0; // feedforward term
        this.driveMotor.getConfigurator().apply(driveConfigs);
    }

    public void setModuleToEncoder() {
        double encPosition = encoder.getAbsPosition(); // 0.0 to 1.0, inclusive, increasing counterclockwise
        steeringMotor.setPosition(STEERING_GEAR_RATIO * encPosition);
    }

    public void setModuleState(SwerveModuleState state) {
        // get the current position of the steering motor and optimize the state
        // make sure positionOfSteering in [0.0, 1.0)
        double positionOfSteering = steeringMotor.getPosition().getValueAsDouble() / STEERING_GEAR_RATIO;
        positionOfSteering -= (long)positionOfSteering;
        if (positionOfSteering < 0) {
            // it seems that optimize wants this always between 0 and 1
            positionOfSteering += 1;
        }
        state.optimize(new Rotation2d(positionOfSteering));

        // set the position of the steering motor
        // remember that angle is the negative of what the motors want, hence the minus
        double positionInRotations = STEERING_GEAR_RATIO * -state.angle.getDegrees() / 360.0;
        ControlRequest steeringControlRequest = new PositionDutyCycle(positionInRotations);
        this.steeringMotor.setControl(steeringControlRequest);

        // set the speed of the drive motor
        double FUDGE_FACTOR = 1.6; // TODO: What is wrong?
        double wheelCircumference = 2 * Math.PI * WHEEL_RADIUS_METERS;
        double wheelRotationsPerSecond = state.speedMetersPerSecond / wheelCircumference;
        double motorRotationsPerSecond = wheelRotationsPerSecond * DRIVE_GEAR_RATIO * FUDGE_FACTOR;
        ControlRequest driveControlRequest = new VelocityDutyCycle(motorRotationsPerSecond);
        this.driveMotor.setControl(driveControlRequest);

        // this is probably not the best place for this code, but this is a sandbox project

        if (OUTPUT_TO_SMART_DASH) {
            SmartDashboard.putNumber(
                "Swerve States/" + this.steeringMotorCANId + "/Drive/demand_wheelRotationsPerSecond", wheelRotationsPerSecond);
            
            SmartDashboard.putNumber(
                "Swerve States/" + this.steeringMotorCANId + "/Steering/demand_positionInRotations", positionInRotations);
            SmartDashboard.putNumber(
                "Swerve States/" + this.steeringMotorCANId + "/Steering/actual_positionOfSteering", positionOfSteering);

            double encPosition = encoder.getAbsPosition(); // 0.0 to 1.0, inclusive, increasing counterclockwise
            SmartDashboard.putNumber(
                "Swerve States/" + this.steeringMotorCANId + "/external_encoderPosition", encPosition);        
        }
    }
}
