package frc.robot.subsystems;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private static final double WHEEL_RADIUS_METERS = 1.0;

    private final TalonFX steeringMotor;
    private final TalonFX driveMotor;
  
    public SwerveModule(int steeringMotorCANId, int driveMotorCANId) {
        this.steeringMotor = new TalonFX(steeringMotorCANId);
        this.driveMotor = new TalonFX(driveMotorCANId);
    }

    public void setModuleState(SwerveModuleState state) {
        // TODO IMPORTANT: We need to "zero" the module and factor that in
        // TODO IMPORTANT: This doesn't work because PositionDutyCycle is in ticks, I think

        // set the position of the steering motor
        ControlRequest steeringControlRequest = new PositionDutyCycle(state.angle.getRadians());
        this.steeringMotor.setControl(steeringControlRequest);

        // set the speed of the drive motor
        double angularVelocity = state.speedMetersPerSecond / WHEEL_RADIUS_METERS;
        ControlRequest driveControlRequest = new VelocityDutyCycle(angularVelocity);
        this.driveMotor.setControl(driveControlRequest);
    }
}
