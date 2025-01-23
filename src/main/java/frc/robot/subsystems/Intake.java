package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
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

    public boolean canintakeDecend = true;

    public boolean canintakeAscend = true;

    private final RelativeEncoder intakeEncoderActuation = intakeActuation.getEncoder();

    private final RelativeEncoder intakeEncoderWheels = intakeWheels.getEncoder();

    private final SparkMaxConfig intakeActuatioConfig = new SparkMaxConfig();

    private final SparkMaxConfig intakeWheelsConfig = new SparkMaxConfig();


    public Intake() {
        intakeActuation.configure(intakeActuatioConfig, null, null);
        intakeWheels.configure(intakeWheelsConfig, null, null);

        setUpMotors();
    }

    private void setUpMotors() {
        intakeEncoderActuation.setPosition(0);
        intakeEncoderWheels.setPosition(0);
    }

    public void goup(double speed) {
        if (canintakeAscend) {
            intakeActuation.set(speed);
        } else {
            intakeActuation.set(0);
        }
    }

    public void godown(double speed) {
        if (canintakeDecend) {
            intakeActuation.set(-speed);
        } else {
            intakeActuation.set(0);
        }
    }

    public void intakeCoral(double speed) {
        intakeWheels.set(speed);
    }
}
