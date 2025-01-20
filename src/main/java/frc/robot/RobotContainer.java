// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
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

	Joystick joystick = new Joystick(OperatorConstants.driverControllerPort);
	XboxController xboxController = new XboxController(1);

	ControllerInput controller = new ControllerInput(xboxController);
	Vision visionSystem = new Vision(Constants.VisionConstants.ipAddress, Constants.VisionConstants.CameraRotations, Constants.VisionConstants.apriltagAngles); 

	public Swerve swerve = new Swerve(controller, visionSystem);


	Auto auto = new Auto(swerve);

	final AutoChooser autoChooser;


	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {

		autoChooser = new AutoChooser();

		autoChooser.addRoutine("Figure8", auto::figure8);
		autoChooser.addRoutine("MiniFigure8", auto::miniFigure8);

		SmartDashboard.putData(autoChooser);

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

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	//public Command getAutonomousCommand() {
	// An example command will be run in autonomous
	// return Autos.exampleAuto(m_exampleSubsystem);
	// }
}
