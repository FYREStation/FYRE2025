package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;

/**
 * Contains all control mechanisms for the intake.
 */
public class ClimberControl extends Command {
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

    private Climber climber;

    /**
     * Initializes a new intake controller command base.

     * @param subsystem - The climber subsystem to run off of.
     */
    public ClimberControl(Climber subsystem) {
        this.climber = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {

    }


    public Command pinch = Commands.runOnce(() -> {
        climber.climb(speed);
    });
    public Command release = Commands.runOnce(() -> {
        climber.unClimb(speed);
    });
    public Command stopClimber = Commands.runOnce(() -> {
        climber.stopClimber();

    });
}
