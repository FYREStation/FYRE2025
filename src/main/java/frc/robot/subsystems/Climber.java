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
import frc.robot.Constants.ClimberConstants;

/**
 * All of the interfaces for the physical climber.
 */
public class Climber extends SubsystemBase {
    private final SparkMax climberMotor = new SparkMax(
        ClimberConstants.climberMotorPort, 
        SparkLowLevel.MotorType.kBrushed
    );

    private double rotationsToBottom = ClimberConstants.rotationsToBottom;

    private final RelativeEncoder climberEncoder = climberMotor.getEncoder();

    private final SparkMaxConfig climberMotorConfig = new SparkMaxConfig();

    /**
     * Constructs the climber subsystem and initializes the motor and encoder.
     */
    public Climber() {
        setUpMotor();

        climberMotor.configure(
            climberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void setUpMotor() {
        climberEncoder.setPosition(0);
        // climberWheelEncoder.setPosition(0);  // This position doesn't really matter at all ¯\_(ツ)_/¯ If i'm wrong, uncomment :)
    }

    /* Actuates the climber at a given speed (pinches) */
    public void climb (double speed) {
        climberMotor.set(speed);
    }

    /* Actuates the climber at a given speed (releases) */
    public void unClimb(double speed) {
        climberMotor.set(-speed);
    }
    
    public void stopClimber() {
        climberMotor.set(0);

    }
}
