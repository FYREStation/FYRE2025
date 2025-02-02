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
import frc.robot.Constants.IntakeConstants;

/**
 * All of the interfaces for the physical intake.
 */
public class Intake extends SubsystemBase {
    private final SparkMax intakeWheels = new SparkMax(
        IntakeConstants.intakeWheelPort, 
        SparkLowLevel.MotorType.kBrushed
    );

    // The neo motor that will handle the intake actuation.
    private final SparkMax intakeActuation = new SparkMax(
        IntakeConstants.intakeActuationPort,
        SparkLowLevel.MotorType.kBrushed
    );

    private double rotationsToBottom = IntakeConstants.rotationsToBottom;

    public boolean canintakeDecend = true; //may need to be changed to false in code

    public boolean canintakeAscend = true; //may need to be changed to false in code

    private final RelativeEncoder intakeEncoderActuation = intakeActuation.getEncoder();

    private final RelativeEncoder intakeEncoderWheels = intakeWheels.getEncoder();

    private final SparkMaxConfig intakeActuatioConfig = new SparkMaxConfig();

    private final SparkMaxConfig intakeWheelsConfig = new SparkMaxConfig();


    /**
     * Constructs the elevator subsystem and inializes motors, controllers, and the like.
     */
    public Intake() {

        setUpMotors();

        intakeActuation.configure(
            intakeActuatioConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeWheels.configure(
            intakeWheelsConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private void setUpMotors() {
        intakeEncoderActuation.setPosition(0);
        intakeEncoderWheels.setPosition(0);
    }

    /**
     * Moves the intake up at a given speed.

     * @param speed - the speed at which to raise the intake
     */
    public void goUp(double speed) {
        if (canintakeAscend) {
            intakeActuation.set(speed);
        } else {
            intakeActuation.set(0);
        }
    }

    /**
     * Moves the intake down at a given speed.

     * @param speed - the speed at which to lower the intake
     */
    public void goDown(double speed) {
        if (canintakeDecend) {
            intakeActuation.set(-speed);
        } else {
            intakeActuation.set(0);
        }
    }

    /**
     * Runs the intake wheels at the given speed.

     * @param speed - the speed at which to run the wheels
     */
    public void intakeCoral(double speed) {
        intakeWheels.set(speed);
    }
}
