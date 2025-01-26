package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ElevatorLiftConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class Arm extends SubsystemBase {
    // The CIM which will be the "leader" for the arm.
    private final SparkMax armMotor = new SparkMax(
        ArmConstants.armPort, 
        SparkLowLevel.MotorType.kBrushless
    );

    private final SparkMaxConfig armMotorConfig = new SparkMaxConfig();

    private final RelativeEncoder armEncoder = armMotor.getEncoder();

    private final ElevatorFeedforward armFeedForward = new ElevatorFeedforward(
        ArmConstants.staticGain,
        ArmConstants.gravityGain,
        ArmConstants.velocityGain
    );

    private double rotationsToTop = ArmConstants.maxRotations;

    private TrapezoidProfile.State topState = new TrapezoidProfile.State( // Caden, may need to use something other than trapazoid
        rotationsToTop,
        0
        );
    
    //put extra defaults here
    
    private TrapezoidProfile.State bottomState = new TrapezoidProfile.State(
        0,
        0
        );

        private boolean manualOverride = false;
        private boolean canMoveUp = true;
        private boolean canMoveDown = true;
        private boolean isCalibrating = false;

    private final ProfiledPIDController armController = new ProfiledPIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD,
        new TrapezoidProfile.Constraints(
            ArmConstants.maxVelocity,
            ArmConstants.maxAcceleration
        )
    );

    public Arm() {
        armMotor.configure(armMotorConfig, null, null);
        setUpMotors();
    }

    
    
}