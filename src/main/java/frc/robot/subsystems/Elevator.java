// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ElevatorLiftConstants;

/** The elevator subsystem to be used by any elevator commands. */
public class Elevator extends ProfiledPIDSubsystem {

    // The CIM which will be the "leader" for the elevator lift.
    private final CANSparkMax elevatorMotor1 = new CANSparkMax(
        ElevatorLiftConstants.elevatorMotor1Port, 
        CANSparkLowLevel.MotorType.kBrushless //needs to be updated
    );

    // The other CIM on the elevator lift which will follow the main motor.
    private final CANSparkMax elevatorMotor2 = new CANSparkMax(
        ElevatorLiftConstants.elevatorMotor2Port,
        CANSparkLowLevel.MotorType.kBrushless //needs to be updated
    );

    private final ElevatorFeedforward ElevatorFeedforward = new ElevatorFeedforward(
        ElevatorLiftConstants.staticGain,
        ElevatorLiftConstants.gravityGain,
        ElevatorLiftConstants.velocityGain,
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

    private TrapezoidProfile topState = new TrapezoidProfile(
        rotationsToTop,
        0,
        );
    
    //put extra defaults here
    
    private TrapezoidProfile bottomState = new TrapezoidProfile(
        0,
        0,
        );

    // Variable to control if the driver needs to manually ovveride the elevator
    private boolean manualOverride = false;

    //Variables to keep track of if elevator can move up and down

    private boolean canMoveUp = null;
    
    private boolean canMoveDown = null;

    //variable to keep track if the elevator is currently calibratig
    private boolean isCalibrating = false;

    // constructor
    public Elevator() {
        super(
            new ProfiledPIDController(
                ElevatorLiftConstants.kP,
                ElevatorLiftConstants.kI,
                ElevatorLiftConstants.kD,
                new TrapezoidProfile.Constraints(
                    ElevatorLiftConstants.maxVelocity,
                    ElevatorLiftConstants.maxAcceleration
                )
            )
        );
        setupMotors();
    }

    @ovveride  
    public void periodic() {
        // This method will be called once per scheduler run
        // check if the controller is not yet at it's goal and the manual override is not active
        if (!(super.getController().atGoal() || manualOverride || isCalibrating)) {
            // set the setpoint to the controller
            elevatorMotor1.set(
                super.getController().calculate(
                    getEncoderDistances()
                    super.getController().getGoal()
                )
            );
        }
    }
    //sets up the motors at the beginning of the program

    private void setUpMotors() {
        elevatorMotor2.follow(elevatorMotor1, true);

        resetEncoders();
    }

    //sets the goal to the top state of the elevator

    public void goToTop() {
        setGoal(topState);
        enable();
    
    public void goToBottom() {
        setGoal(bottomState);
        enable();
    }

    //runs motors down

    
}