package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

/**
 * Contains all control mechanisms for the intake.
 */
public class IntakeControl extends Command {

    private Intake intake;

    /**
     * Initializes a new intake controller command base.

     * @param subsystem - The Intake subsystem to run off of.
     */
    public IntakeControl(Intake subsystem) {
        this.intake = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {}

    public Command intakeUp = Commands.runOnce(() -> {
        intake.goUp(IntakeConstants.intakeActuationThrottle);
    });

    public Command intakeDown = Commands.runOnce(() -> {
        intake.goDown(IntakeConstants.intakeActuationThrottle);
    });

    public Command intakeCoral = Commands.runOnce(() -> {
        intake.intakeCoral(IntakeConstants.intakeWheelThrottle);
    });
}
