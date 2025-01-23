package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;
public class Intakething extends Command {
    private Intake intake;
    /**
     * Initializes a new intake controller command base.
     *
     * @param subsystem - The Intake subsystem to run off of.
     */
    public Intakething(Intake subsystem) {
        this.intake = subsystem;
        addRequirements(intake);
    }
    @Override
    public void execute() {

    }
    /**
     * Rotates the intake at the given speed for the given ammount of seconds.

     * @param seconds - the amount of seconds to run the intake
     * @param time - the amount of time the intake has been running 
     * @param speed - the speed to rotate the actuation
     * @param wheelSpeed - the speed to rotate flywheels
     */
    //PLACEHOLDER VALUES
    int speed = 5;
    int wheelSpeed = 5;
    public Command intakeUp = Commands.runOnce(() -> {
        intake.goUp(speed);
    });
    public Command intakeDown = Commands.runOnce(() -> {
        intake.goDown(speed);
    });
    public Command intakeCoral = Commands.runOnce(() -> {
        intake.intakeCoral(wheelSpeed);;
    });
}
