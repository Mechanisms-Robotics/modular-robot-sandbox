package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class Outtake extends SubsystemBase {

    //update this CAN ID to match the hardware
    private static final int OUTTAKE_MOTOR_CAN_ID = 20;

    private static final SparkMax motor = new SparkMax(OUTTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    /** Creates a new Outtake. */
    public Outtake() {

    }

    //TODO: add a set method to make the motor go brrr and stop

    public final Command startOuttake(Outtake outtake) {
        return new InstantCommand(() -> motor.set(1));
    }
   
    public final Command stopOuttake(Outtake outtake) {
        return new InstantCommand(() -> motor.set(0));
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}
