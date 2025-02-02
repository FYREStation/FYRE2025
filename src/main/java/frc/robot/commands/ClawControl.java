package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Claw;

/**
 * Contains all control mechanisms for the intake.
 */
public class ClawControl extends Command {
    /**
     * Rotates the intake at the given speed for the given ammount of seconds.

     * @param seconds - the amount of seconds to run the intake
     * @param time - the amount of time the intake has been running 
     * @param speed - the speed to rotate the actuation
     * @param wheelSpeed - the speed to rotate flywheels
     */
    
    //PLACEHOLDER VALUES
    int speed = -1234567890;
    int wheelSpeed = -1234567890;

    private Claw claw;

    /**
     * Initializes a new intake controller command base.

     * @param subsystem - The claw subsystem to run off of.
     */
    public ClawControl(Claw subsystem) {
        this.claw = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {

    }


    public Command pinch = Commands.runOnce(() -> {
        claw.pinch(speed);
    });
    public Command release = Commands.runOnce(() -> {
        claw.release(speed);
    });
    public Command stopClaw = Commands.runOnce(() -> {
        claw.stopClaw();
    });
    public Command intake = Commands.runOnce(() -> {
        claw.intake(wheelSpeed); //TODO: probably need to pinch the claw by some amount to ensure adequate friction
    });
    public Command output = Commands.runOnce(() -> {
        claw.output(wheelSpeed); //TODO: probably need to release the claw by some amount
    });
    public Command stopWheels = Commands.runOnce(() -> {
        claw.stopWheels();
    });
}
