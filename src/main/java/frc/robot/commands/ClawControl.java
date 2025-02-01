package main.java.frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Claw;

/**
 * Contains all control mechanisms for the intake.
 */
public class ClawControl extends Command {

    //PLACEHOLDER VALUES
    int speed = 5;
    int wheelSpeed = 5;

    private Claw claw;

    /**
     * Initializes a new intake controller command base.

     * @param subsystem - The Intake subsystem to run off of.
     */
    public IntakeControl(Claw claw) {
        this.claw = subsystem;
        addRequirements(claw);
    }

    @Override
    public void execute() {

    }


    // TODO: what is this javadoc????
    /**
     * Rotates the intake at the given speed for the given ammount of seconds.

     * @param seconds - the amount of seconds to run the intake
     * @param time - the amount of time the intake has been running 
     * @param speed - the speed to rotate the actuation
     * @param wheelSpeed - the speed to rotate flywheels
     */
    


    public Command intakeUp = Commands.runOnce(() -> {
        intake.goIn(speed);
    });
    public Command intakeDown = Commands.runOnce(() -> {
        intake.goOut(speed);
    });
    public Command intakeCoral = Commands.runOnce(() -> {
        intake.stopClaw();;
    });
}
