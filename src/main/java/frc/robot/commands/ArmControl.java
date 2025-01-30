package main.java.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;

/** The elevator lifting functionality for our arm. */
public class ArmControl extends Command {

    // The elevator subsystem
    private Arm arm;

    // The state of the calibration sequence
    private boolean isCalibrating = false;

    // sets the elevator calibation states
    boolean step1 = false;
    boolean step2 = false;
    boolean step3 = false;

    /**
     * Creates a new elevator command.

     * @param subsystem - the elevator subsystem
     */
    public ArmControl(Arm subsystem) {
        this.elevator = subsystem;
        addRequirements(subsystem);
    }

    /**
     * Called repeatedly when a command is scheduled.
     */
    @Override
    public void execute() {
        if (isCalibrating) {
            if (!step1) {
                step1 = arm.calibrateStep1();
            } else if (step1 && !step2) {
                step2 = arm.calibrateStep2();
            } else if (step2 && !step3) {
                step3 = arm.calibrateStep3();
            }
            isCalibrating = !step3;
            arm.setCalibrating(isCalibrating);
        }
    }

    /**
     * Calibrates the arm.
     */
    public Command calibrateLiftBounds = Commands.runOnce(() -> {
        isCalibrating = true;
        arm.setCalibrating(isCalibrating);
    });

    public Command stopCalibration = Commands.runOnce(() -> {
        isCalibrating = false;
        arm.setCalibrating(isCalibrating);
    });

    /**
     * Sends the ar, to top using PID.
     */
    public Command goToTop = Commands.runOnce(() -> {
        arm.goToTop();
    });


    /**
     * Sends the arm to the bottom using PID.
     */
    public Command goToBottom = Commands.runOnce(() -> {
        arm.goToBottom();
    });

    /**
     * Runs the arm up.
     */
    public Command runMotorForward = Commands.runOnce(() -> {
        arm.runMotorForward();
    });

    /**
     * runs the arm down.
     */
    public Command runMotorReverse = Commands.runOnce(() -> {
        arm.runMotorBackward();
    });

    public Command toggleManualOverride = Commands.runOnce(() -> {
        arm.toggleManualOverride();
    });

    /**
     * Stops motors using the PID stop and the motor stop.
     */
    public Command stopMotors = Commands.runOnce(() -> {
        arm.stopMotor();
    });
}