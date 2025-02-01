package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

/**
 * All of the interfaces for the physical claw.
 */
public class Claw extends SubsystemBase {
    private final SparkMax clawMotor = new SparkMax(
        ClawConstants.clawMotorPort, 
        SparkLowLevel.MotorType.kBrushed
    );
    private final SparkMax clawWheelMotor = new SparkMax(
        ClawConstants.clawWheelMotorPort, 
        SparkLowLevel.MotorType.kBrushed
    );

    private double rotationsToBottom = ClawConstants.rotationsToBottom;

    private final RelativeEncoder clawEncoder = clawMotor.getEncoder();

    private final SparkMaxConfig clawMotorConfig = new SparkMaxConfig();

    private final RelativeEncoder clawWheelEncoder = clawMotor.getEncoder();

    private final SparkMaxConfig clawWheelMotorConfig = new SparkMaxConfig();
    /**
     * Constructs the claw subsystem and initializes the motor and encoder.
     */
    public Claw() {
        setUpMotor();

        clawMotor.configure(
            clawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        clawWheelMotor.configure(
            clawWheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setUpMotor() {
        clawEncoder.setPosition(0);
        // clawWheelEncoder.setPosition(0);  // This position doesn't really matter at all ¯\_(ツ)_/¯ If i'm wrong, uncomment :)
    }

    /* Actuates the claw at a given speed (pinches) */
    public void pinch(double speed) {
        clawMotor.set(speed);
    }

    /* Actuates the claw at a given speed (releases) */
    public void release(double speed) {
        clawMotor.set(-speed);
    }
    
    public void stopClaw() {
        clawMotor.set(0);
    }
    /* Runs the intake wheels at the given speed.*/
    public void intake(double speed) {
        clawWheelMotor.set(speed);
    }

    public void output(double speed) {
        clawWheelMotor.set(-speed);
    }

    public void stopWheels() {
        clawWheelMotor.set(0);
    }
}
