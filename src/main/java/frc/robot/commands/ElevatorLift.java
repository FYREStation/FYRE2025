package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;

/** The elevator lifting functionality for our arm. */
public class ElevatorLift extends Command {

    // The elevator subsystem
    private Elevator elevator;

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
    public ElevatorLift(Elevator subsystem) {
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
                step1 = elevator.calibrateStep1();
            } else if (step1 && !step2) {
                step2 = elevator.calibrateStep2();
            } else if (step2 && !step3) {
                step3 = elevator.calibrateStep3();
            }
            isCalibrating = !step3;
            elevator.setCalibrating(isCalibrating);
        }
    }

    /**
     * Calibrates the elevator.
     */
    public Command calibrateLiftBounds = Commands.runOnce(() -> {
        isCalibrating = true;
        elevator.setCalibrating(isCalibrating);
    });

    public Command stopCalibration = Commands.runOnce(() -> {
        isCalibrating = false;
        elevator.setCalibrating(isCalibrating);
    });

    /**
     * Sends the elevator to top using PID.
     */
    public Command goToTop = Commands.runOnce(() -> {
        elevator.goToTop();
    });


    /**
     * Sends the elevator to the bottom using PID.
     */
    public Command goToBottom = Commands.runOnce(() -> {
        elevator.goToBottom();
    });

    /**
     * Runs the elevator up.
     */
    public Command runMotorForward = Commands.runOnce(() -> {
        elevator.runMotorForward();
    });

    /**
     * runs the elevator down.
     */
    public Command runMotorReverse = Commands.runOnce(() -> {
        elevator.runMotorBackward();
    });

    public Command toggleManualOverride = Commands.runOnce(() -> {
        elevator.toggleManualOverride();
    });

    /**
     * Stops motors using the PID stop and the motor stop.
     */
    public Command stopMotors = Commands.runOnce(() -> {
        elevator.stopMotor();
    });
}