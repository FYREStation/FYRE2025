// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmControl;
import frc.robot.commands.ClawControl;
import frc.robot.commands.ClimberControl;
import frc.robot.commands.ElevatorLift;
import frc.robot.commands.IntakeControl;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.Auto;
import frc.robot.util.ControllerInput;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	
    CommandJoystick joystick = new CommandJoystick(OperatorConstants.operatorControllerPort);
    CommandXboxController xboxController = new CommandXboxController(OperatorConstants.driverControllerPort);

    ControllerInput controller = new ControllerInput(xboxController);
    Vision visionSystem = new Vision(
        Constants.VisionConstants.ipAddress, 
        Constants.VisionConstants.CameraRotations, 
        Constants.VisionConstants.apriltagAngles); 

    public Swerve swerve = new Swerve(controller, visionSystem);

    /*
    public Intake intake = new Intake();
    public IntakeControl intakeControl = new IntakeControl(intake);
    */

    public Elevator elevator = new Elevator();
    public ElevatorLift elevatorControl = new ElevatorLift(elevator);

    public Arm arm = new Arm();
    public ArmControl armControl = new ArmControl(arm);

    public Claw claw = new Claw();
    public ClawControl clawControl = new ClawControl(claw);

    public Climber climber = new Climber();
    public ClimberControl climberControl = new ClimberControl(climber);

    Auto auto = new Auto(swerve);
    final AutoChooser autoChooser;

    /**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
    public RobotContainer() {

        // intake.setDefaultCommand(intakeControl);

        autoChooser = new AutoChooser();

        autoChooser.addRoutine("Figure8", auto::figure8);
        autoChooser.addRoutine("MiniFigure8", auto::miniFigure8);
        autoChooser.addRoutine("Dummy1", auto::real);

        SmartDashboard.putData(autoChooser);

        autoChooser.select("Dummy1");

        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

        // Configure the trigger bindings
        configureBindings();
		
    }

    /**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
    private void configureBindings() {

        // driver bindings 
        xboxController.start()
            .onChange(controller.toggleNos);

        xboxController.leftTrigger(0.75)
            .onChange(controller.toggleFeildRelative);

        xboxController.a()
            .onChange(controller.toggleLockOn);

        xboxController.b()
            .onChange(controller.toggleAlignTag);

        // manipulator bindings
        joystick.button(1)
            .onTrue(clawControl.intake)
            .onFalse(clawControl.stopWheels);

        joystick.button(2)
            .onTrue(clawControl.output)
            .onFalse(clawControl.stopWheels);

        joystick.button(7)
            .onTrue(elevatorControl.goToTop);
            // .onTrue(elevatorControl.runMotorForward)
            // .onFalse(elevatorControl.stopMotors);
        
        joystick.button(11)
            .onTrue(elevatorControl.goToBottom);
            // .onTrue(elevatorControl.runMotorReverse)
            // .onFalse(elevatorControl.stopMotors);

        joystick.button(5)
            .onTrue(armControl.goToTop);
            // .onTrue(armControl.runMotorForwards)
            // .onFalse(armControl.stopMotors);

        joystick.button(3)
            .onFalse(armControl.goToBottom);
            // .onTrue(armControl.runMotorBackward)
            // .onFalse(armControl.stopMotors);

        joystick.button(6)
            .onTrue(climberControl.pinch)
            .onFalse(climberControl.stopClimber);

        joystick.button(4)
            .onTrue(climberControl.release)
            .onFalse(climberControl.stopClimber);

    }

    /**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.selectedCommand();
    }
}
