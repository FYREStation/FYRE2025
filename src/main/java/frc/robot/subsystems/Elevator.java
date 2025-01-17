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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorLiftConstants;

/** The elevator subsystem to be used by any elevator commands. */
public class Elevator extends ProfiledPIDController {

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

    private final ElevatorFeedforward ElevatorFeedforward = new ElevatorFeedforward(
        ElevatorLiftConstants.staticGain,
        ElevatorLiftConstants.gravityGain,
        ElevatorLiftConstants.velocityGain
    );
    //The encoder on one of the elevator cims
    private final RelativeEncoder elevatorEncoder1 = elevatorMotor1.getEncoder();
    //Encoder on the other one
    private final RelativeEncoder elevatorEncoder2 = elevatorMotor2.getEncoder();

    // The bottom limit switch for the elevator lift.
    private final DigitalInput bottomLimitSwith = new DigitalInput(
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

    private boolean canMoveUp = false;
    
    private boolean canMoveDown = false;

    //variable to keep track if the elevator is currently calibratig
    private boolean isCalibrating = false;

    // constructor
    public Elevator() {
        super(
            ElevatorLiftConstants.kP, 
            ElevatorLiftConstants.kI, 
            ElevatorLiftConstants.kD, 
            new TrapezoidProfile.Constraints(
                ElevatorLiftConstants.maxVelocity, 
                ElevatorLiftConstants.maxAcceleration
            )
        );

        setUpMotors();
        
    }

    @ovveride  
    public void periodic() { //Vibhav to Caden::: Check If this is the correct way to check if the controller is at it's goal
        // This method will be called once per scheduler run
        // check if the controller is not yet at it's goal and the manual override is not active
        if (!(super.atGoal() || manualOverride || isCalibrating)) { 
            // set the setpoint to the controller
            elevatorMotor1.set(
                super.calculate(
                    getEncoderDistances(),
                    getGoal().position
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
        setGoal(topState);
        enable();
    }
    
    public void goToBottom() {
        setGoal(bottomState);
        enable();
    }

    public double getEncoderDistances() {
        return (elevatorEncoder1.getPosition() + elevatorEncoder2.getPosition()) / 2;

    }

    private void resetEncoders() {
        elevatorEncoder1.setPosition(0.0);
        elevatorEncoder2.setPosition(0.0);
    }

}


    //runs motors down

    
