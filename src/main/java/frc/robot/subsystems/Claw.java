package main.java.frc.robot.subsystems;

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

    private double rotationsToBottom = ClawConstants.rotationsToBottom;

    private final RelativeEncoder clawEncoder = clawMotor.getEncoder();

    private final SparkMaxConfig clawMotorConfig = new SparkMaxConfig();

    /**
     * Constructs the claw subsystem and initializes the motor and encoder.
     */
    public Claw() {
        setUpMotor();

        clawMotor.configure(
            clawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setUpMotor() {
        clawEncoder.setPosition(0);
    }

    /**
     * Moves the Claw up at a given speed.

     * @param speed - the speed at which to raise the Claw
     */
    public void goIn(double speed) {
        clawMotor.set(speed);
    }

    /**
     * Moves the intake down at a given speed.

     * @param speed - the speed at which to lower the intake
     */
    public void goOut(double speed) {
        clawMotor.set(-speed);
    }

    /**
     * Runs the intake wheels at the given speed.

     * @param speed - the speed at which to run the wheels
     */
    public void stopClaw() {
        clawMotor.set(0);
    }
}
