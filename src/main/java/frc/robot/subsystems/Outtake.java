package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Outtake extends SubsystemBase {

    private static final int OUTTAKE_MOTOR_CAN_ID = 20; // TODO: update this CAN ID to match the HW
    private static final double OUTTAKE_POWER = 0.5; // adjust this value as needed between -1 and 1

    private static final SparkMax motor = new SparkMax(OUTTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    /** Creates a new Outtake. */
    public Outtake() {

        // TODO: we may need to set the idle mode to Brake if we have a problem with coral
        // rolling out of the outtake before we run the motor

        // initiatlize the motor to be off
        this.setPower(0);

    }

    public void setPower(double desiredPower) {

        // spin the motor with the desired power
        // NOTE: specifying power and using dutyCycle control may not be the most
        // accurate with repeatable results, but is probably sufficient for our simple
        // outtake mechanism...especially with gravity on our side.
        motor.getClosedLoopController().setReference(desiredPower, ControlType.kDutyCycle);
    
        // Optionally output to SmartDashboard for monitoring
        if (Constants.OUTPUT_TO_SMART_DASH) {
            SmartDashboard.putNumber("Outtake/" + this.OUTTAKE_MOTOR_CAN_ID + "/desiredPower", desiredPower);
        }
    }

    // Command Factory which returns a command to run the outtake motor
    public Command runOuttakeCommand() {
        // The second argument to RunCommand lists the subsystems required for this command, in
        // this case, the Outtake subsystem, which is referenced by "this". This ensures that
        // no other command which requires the Outtake subsystem can run at the same time.
        return new RunCommand(() -> setPower(OUTTAKE_POWER), this);
    }

    // Command Factory which returns a command to stop the outtake motor
    public Command stopOuttakeCommand() {
        return new InstantCommand(() -> setPower(0), this);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
