package frc.robot.util;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class Auto {
    
    Swerve swerve;

	AutoFactory autoFactory;

    public Auto(Swerve swerve) {

        this.swerve = swerve;

        autoFactory = new AutoFactory(
			swerve::getPose,
			swerve::resetOdometry, 
            swerve::followTrajectory, 
            true, 
            swerve
        );
    }

    // routines need to be loaded at start time - not runtime before we're about to use it
    // loading routines takes time so don't do it when enabled
    public AutoRoutine dummyRoutine() {
        AutoRoutine exampleRoutine = autoFactory.newRoutine("example");

        AutoTrajectory exampleTrajectory = exampleRoutine.trajectory("exampleTrajectory");
        AutoTrajectory exampleTrajectory2 = exampleRoutine.trajectory("exampleTrajectory2");

        exampleRoutine.active().onTrue(
            Commands.sequence(
                exampleTrajectory.resetOdometry(),
                exampleTrajectory.cmd()
            )
        );

        // we can define auto routines PROGRAMATICALLY!!!

        // exampleTrajectory.atTime("exampleEvent").onTrue(/* some command here */);
        // exampleTrajectory.done().onTrue(exampleTrajectory2.cmd());

        // exampleTrajectory2.active().whileTrue(/* some other command here */);
        // exampleTrajectory2.done().onTrue(/* another command here */);

        return exampleRoutine;
    }
}
