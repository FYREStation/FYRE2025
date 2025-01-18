// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorLiftConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** The elevator subsystem to be used by any elevator commands. */
public class Elevator extends SubsystemBase {

    // The CIM which will be the "leader" for the elevator lift.
    private final SparkMax elevatorMotor1 = new SparkMax(
        ElevatorLiftConstants.elevatorMotor1Port, 
        SparkLowLevel.MotorType.kBrushless //needs to be updated
    );

    private final SparkMaxConfig elevatorMotor1Config = new SparkMaxConfig();

    // The other CIM on the elevator lift which will follow the main motor.
    private final SparkMax elevatorMotor2 = new SparkMax(
        ElevatorLiftConstants.elevatorMotor2Port,
        SparkLowLevel.MotorType.kBrushless //needs to be updated
    );

    private final SparkMaxConfig elevatorMotor2Config = new SparkMaxConfig();

    private final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(
        ElevatorLiftConstants.staticGain,
        ElevatorLiftConstants.gravityGain,
        ElevatorLiftConstants.velocityGain
    );
    //The encoder on one of the elevator cims
    private final RelativeEncoder elevatorEncoder1 = elevatorMotor1.getEncoder();
    //Encoder on the other one
    private final RelativeEncoder elevatorEncoder2 = elevatorMotor2.getEncoder();

    // The bottom limit switch for the elevator lift.
    private final DigitalInput bottomLimitSwitch = new DigitalInput(
        ElevatorLiftConstants.bottomLimitSwitchPort
    );

    // The top limit switch for the elevator lift.
    private final DigitalInput topLimitSwitch = new DigitalInput(
        ElevatorLiftConstants.topLimitSwitchPort
    );

    //the variable that will be used to calculate the maxiumum rotations tot hte top of the elevator

    private double rotationsToTop = ElevatorLiftConstants.maxRotations;

    private TrapezoidProfile.State topState = new TrapezoidProfile.State(
        rotationsToTop,
        0
        );
    
    //put extra defaults here
    
    private TrapezoidProfile.State bottomState = new TrapezoidProfile.State(
        0,
        0
        );

    // Variable to control if the driver needs to manually ovveride the elevator
    private boolean manualOverride = false;

    //Variables to keep track of if elevator can move up and down

    private boolean canMoveUp = true; //Needs to be updated in runtime somehow
    
    private boolean canMoveDown = false; //Needs to be updated in runtime somehow

    //variable to keep track if the elevator is currently calibratig
    private boolean isCalibrating = false;

    private final ProfiledPIDController controller =
    new ProfiledPIDController(
        ElevatorLiftConstants.kP,
        ElevatorLiftConstants.kI,
        ElevatorLiftConstants.kD,
        new TrapezoidProfile.Constraints(
          ElevatorLiftConstants.maxVelocity,
          ElevatorLiftConstants.maxAcceleration));


    // constructor
    public Elevator() {

        setUpMotors();
        
    }

    public void periodic() { //Vibhav to Caden::: Check If this is the correct way to check if the controller is at it's goal
        // This method will be called once per scheduler run
        // check if the controller is not yet at it's goal and the manual override is not active
        if (!(controller.atGoal() || manualOverride || isCalibrating)) { 
            // set the setpoint to the controller
            elevatorMotor1.set(
                controller.calculate(
                    getEncoderDistances(),
                    controller.getGoal().position
                )
            );
        }
    }
    //sets up the motors at the beginning of the program

    private void setUpMotors() {
        

        elevatorMotor2Config.follow(0); //Vibhav to Caden::: Check if this is the correct way to follow the leader motor

        resetEncoders();
    }

    //sets the goal to the top state of the elevator

    public void goToTop() {
        controller.setGoal(topState);
    }
    
    public void goToBottom() {
        controller.setGoal(bottomState);
    }

    public double getEncoderDistances() {
        return (elevatorEncoder1.getPosition() + elevatorEncoder2.getPosition()) / 2;

    }

    private void resetEncoders() {
        elevatorEncoder1.setPosition(0.0);
        elevatorEncoder2.setPosition(0.0);
    }

    public void runMotorForward() {
        if (canMoveUp) {
            elevatorMotor1.set(ElevatorLiftConstants.elvevatorThrottle);
        } else {
            elevatorMotor1.stopMotor();
        }
    }

    public void runMotorBackward() {
        if (canMoveDown) {
            elevatorMotor1.set(-ElevatorLiftConstants.elvevatorThrottle);
        } else {
            elevatorMotor1.stopMotor();
        }
    }

    public void stopMotor() {
        elevatorMotor1.stopMotor();
    }

        /**
     * Returns the state of the botom limit switch on the elevator
     * This will return true if it is being triggered, and false if not.

     * @return switch value - the value of the limit switch
     */
    public boolean getBottomSwitch() {
        return bottomLimitSwitch.get();
    }

    /**
     * Returns the state of the top limit switch on the elevator
     * This will return true if it is being triggered, and false if not.

     * @return switch value - the value of the limit switch
     */
    public boolean getTopSwitch() {
        return topLimitSwitch.get();
    }

    /**
     * Returns the top state of the elevator.

     * @return topState - the top state of the elevator
     */
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
            elevatorMotor1.set(-0.1);
            return false;
        }
        elevatorMotor1.stopMotor();

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
            elevatorMotor1.set(0.1);
            return false;
        }
        elevatorMotor1.stopMotor();

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
            elevatorMotor1.set(-0.1);
            return false;
        }
        elevatorMotor1.stopMotor();
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
        elevatorMotor1.setVoltage(output + feedforward);
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




    //runs motors down

    
