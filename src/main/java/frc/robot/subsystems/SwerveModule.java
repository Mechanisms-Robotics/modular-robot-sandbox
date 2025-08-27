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

    private final int steeringMotorCANId; // FOR TEST PURPOSES ONLY

    private final TalonFX steeringMotor;
    private final TalonFX driveMotor;

    private final Canandmag encoder;
  
    public SwerveModule(int steeringMotorCANId, int driveMotorCANId, int encoderCANId) {
        this.steeringMotorCANId = steeringMotorCANId;

        this.steeringMotor = new TalonFX(steeringMotorCANId);
        this.driveMotor = new TalonFX(driveMotorCANId);
        this.encoder = new Canandmag(encoderCANId);

        var configs = new TalonFXConfiguration();
        configs.Slot0.kP = 0.002;
        configs.Slot0.kI = 0.0;
        configs.Slot0.kD = 0.0;
        configs.Slot0.kV = 0.0; // feedforward term
        this.driveMotor.getConfigurator().apply(configs);
    }

    public void setModuleState(SwerveModuleState state) {        
        // set the position of the steering motor
        double positionInRotations = state.angle.getDegrees() / 360.0;
        ControlRequest steeringControlRequest = new PositionDutyCycle(positionInRotations);
        this.steeringMotor.setControl(steeringControlRequest);

        // set the speed of the drive motor
        double angularVelocity = state.speedMetersPerSecond / WHEEL_RADIUS_METERS;
        ControlRequest driveControlRequest = new VelocityDutyCycle(angularVelocity);
        this.driveMotor.setControl(driveControlRequest);

        // Debug code
        SmartDashboard.putNumber(
            "Swerve States/" + this.steeringMotorCANId + "/positionInRotations", positionInRotations);
        SmartDashboard.putNumber(
            "Swerve States/" + this.steeringMotorCANId + "/angularVelocity", angularVelocity);
    }
}
