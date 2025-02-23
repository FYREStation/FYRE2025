package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.Claw;

/**
 * Contains all control mechanisms for the intake.
 */
public class ClawControl extends Command {

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
    public void execute() {}

    public Command intake = Commands.runOnce(() -> {
        claw.intake(ClawConstants.clawMotorSpeed);
    });
    
    public Command output = Commands.runOnce(() -> {
        claw.output(ClawConstants.clawMotorSpeed);
    });

    public Command slowHold = Commands.runOnce(() -> {
        claw.intake(0.25 * ClawConstants.clawMotorSpeed);
    });

    public Command stopWheels = Commands.runOnce(() -> {
        claw.stopWheels();
    });
}
