package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;

public class Outtake extends SubsystemBase {

    //update this CAN ID to match the hardware
    private static final int OUTTAKE_MOTOR_CAN_ID = 20;

    private static final SparkMax motor = new SparkMax(OUTTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    /** Creates a new Outtake. */
    public Outtake() {

    }

    //TODO: add a set method to make the motor go brrr and stop
    public void outtake() {
        motor.set(1);
    }
    public void stop() {
        motor.set(0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
