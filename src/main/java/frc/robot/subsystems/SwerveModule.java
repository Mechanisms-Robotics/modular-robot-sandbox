package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandmag.Canandmag;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private static final double WHEEL_RADIUS_METERS = 0.03; // ESTIMATED
    private static final double STEERING_GEAR_RATIO = 150.0/7.0; // from SDS website
    private static final double DRIVE_GEAR_RATIO = 6.12; // L3  TODO: check we are using L3

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
        steeringConfigs.Slot0.kP = 0.004;
        steeringConfigs.Slot0.kI = 0.0;
        steeringConfigs.Slot0.kD = 0.0;
        steeringConfigs.Slot0.kV = 0.0; // feedforward term
        this.steeringMotor.getConfigurator().apply(steeringConfigs);

        var driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kP = 0.002;
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
        // set the position of the steering motor
        double positionInRotations = STEERING_GEAR_RATIO * state.angle.getDegrees() / 360.0;
        ControlRequest steeringControlRequest = new PositionDutyCycle(positionInRotations);
        this.steeringMotor.setControl(steeringControlRequest);

        // set the speed of the drive motor
        double wheelCircumference = 2 * Math.PI * WHEEL_RADIUS_METERS;
        double wheelRotationsPerSecond = state.speedMetersPerSecond / wheelCircumference;
        double motorRotationsPerSecond = wheelRotationsPerSecond * DRIVE_GEAR_RATIO;
        ControlRequest driveControlRequest = new VelocityDutyCycle(motorRotationsPerSecond);
        this.driveMotor.setControl(driveControlRequest);

        // Debug code

        SmartDashboard.putNumber(
            "Swerve States/" + this.steeringMotorCANId + "/positionInRotations", positionInRotations);
        SmartDashboard.putNumber(
            "Swerve States/" + this.steeringMotorCANId + "/motorRotationsPerSecond", motorRotationsPerSecond);
        
        SmartDashboard.putNumber(
            "Swerve States/" + this.steeringMotorCANId + "/position", steeringMotor.getPosition().getValueAsDouble());
    }
}
