package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorLiftConstants;


/**
 * All of the interfaces for the phyiscal elevator.
 */
public class Arm extends SubsystemBase {
    // The CIM which will be the "leader" for the arm.
    private final SparkMax armMotor = new SparkMax(
        ArmConstants.armPort, 
        SparkLowLevel.MotorType.kBrushless
    );

    private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();

    private final RelativeEncoder armEncoder = armMotor.getEncoder();

    private final ElevatorFeedforward armFeedForward = new ElevatorFeedforward(
        ArmConstants.staticGain,
        ArmConstants.gravityGain,
        ArmConstants.velocityGain
    );

    private double rotationsToTop = ArmConstants.maxRotations;

    // Caden, may need to use something other than trapazoid
    private TrapezoidProfile.State topState = new TrapezoidProfile.State(
        rotationsToTop,
        0
    );
    
    //put extra defaults here
    
    private TrapezoidProfile.State bottomState = new TrapezoidProfile.State(
        0,
        0
    );

    private boolean manualOverride = false;
    private boolean canMoveUp = true;
    private boolean canMoveDown = true;
    private boolean isCalibrating = false;

    private final ProfiledPIDController armController = new ProfiledPIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD,
        new TrapezoidProfile.Constraints(
            ArmConstants.maxVelocity,
            ArmConstants.maxAcceleration
        )
    );

    public Arm() {
        setUpMotors();
        armMotor.configure(armMotorConfig, 
        ResetMode.kResetSafeParameters, 
        PersistMode.kNoPersistParameters);    
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // check if the controller is not yet at it's goal and the manual override is not active
        if (!(controller.atGoal() || manualOverride || isCalibrating)) { 
            // set the setpoint to the controller
            armMotor.set(
                controller.calculate(
                    getEncoderDistances(),
                    controller.getGoal().position
                )
            );
        }
    }

    private void setUpMotors() {
        resetEncoders();
    }

    public void goToTop() {
        controller.setGoal(topState);
    }
    
    //sets the goal to the bottom state of the elevator
    public void goToBottom() {
        controller.setGoal(bottomState);
    }

    public double getEncoderDistances() {
        return (armMotor.getEncoder().getPosition());

    }

    private void resetEncoders() {
        armMotor.setPosition(0.0);
  
    }

    public TrapezoidProfile.State getUpState() {
        return topState;
    }

    /**
     * Returns the bottoms state of the elevator.

     * @return bottomState - the bottom state of the elevator
     */
    public TrapezoidProfile.State getDownState() {
        return bottomState;

    }

    /**
     * Calibrates the elevator by running the motor from the bottom position to the top,
     * and measuring the encoder values from that point.
     */
    public boolean calibrateStep1() {
        // ensures that the encoders are at the bottom of the elevator
        if (!getBottomSwitch()) {
            armMotor.set(-0.1);
            return false;
        }
        armMotor.stopMotor();

        // resets the encoder values at the bottom
        resetEncoders();
        return true;
    }

    /**
     * The second step of the elevator calibration.
     * Causes the elevator to go all the way up to the top

     * @return boolean - whether or not the step has been completed
     */
    public boolean calibrateStep2() {
        // runs the motors to the top of the elevator
        if (!getTopSwitch()) {
            armMotor.set(0.1);
            return false;
        }
        armMotor.stopMotor();

        // saves the rotational value at the top
        rotationsToTop = getEncoderDistances();
        return true;
    }

    /**
     * The third step of the elevator calibration.
     * Causes the elevator to go back down.

     * @return boolean - whether this step has completed or not
     */
    public boolean calibrateStep3() {
        // lowers the elevator back down
        if (!getBottomSwitch()) {
            armMotor.set(-0.1);
            return false;
        }
        armMotor.stopMotor();
        return true;
    }

    public void setCalibrating(boolean isCalibrating) {
        this.isCalibrating = isCalibrating;
    }

    /**
     * Toggles the manaul ovrride control of the elevator.
     */
    public void toggleManualOverride() {
        manualOverride = !manualOverride;
    }

    /**
     * This will take in the output, and a set point,
     * and calculates the amount the motor needs to spin based on this input.
     */
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = elevatorFeedForward.calculate(setpoint.position, setpoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        armMotor.setVoltage(output + feedforward);
    }

    /**
     * Method to be used by the PID controller under the hood,
     * this is not used in our code but it is essential to PID.
     * DO NOT DELETE THIS METHOD
     */
    protected double getMeasurement() {
        // possibly add an offset here? 
        return getEncoderDistances();
    }
}
