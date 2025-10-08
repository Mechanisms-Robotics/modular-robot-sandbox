package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class Outtake extends SubsystemBase {

    //update this CAN ID to match the hardware
    private static final int OUTTAKE_MOTOR_CAN_ID = 20;

    //referance values for duty cycle between -1 and 1
    private static final double OUTTAKE_POWER = 1; //referance for duty cycle
    private static final double REVERSE_OUTTAKE_POWER = -0.5; //referance for duty cycle

    private static final SparkMax motor = new SparkMax(OUTTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    /** Creates a new Outtake. */
    public Outtake() {

    }

    public final void setPower(double desiredPower) {
        motor.getClosedLoopController().setReference(desiredPower, ControlType.kDutyCycle);
        SmartDashboard.putNumber("Outtake/power", desiredPower);
    }

    public final Command startOuttake() {
        return new InstantCommand(() -> this.setPower(OUTTAKE_POWER), this);
    }

    public final Command reverseOuttake() {
        return new InstantCommand(() -> this.setPower(REVERSE_OUTTAKE_POWER), this);
    }
   
    public final Command stopOuttake() {
        return new InstantCommand(() -> this.setPower(0), this);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Outtake/position", motor.getAbsoluteEncoder().getPosition());
    }

}
