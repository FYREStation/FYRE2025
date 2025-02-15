package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;

/**
 * A controller object to handle everything involving user input.
 */
public class ControllerInput extends SubsystemBase {

    /** Enumeration to represent what the robot should be doing with vision. */
    public enum VisionStatus {
        NONE,
        ALIGN_TAG,
        LOCKON
    }

    private double x, y, theta;

    // enables / disables "full throttle" on the drive wheels
    private boolean nos;

    private boolean fieldRelative = true;
    private boolean alignWithTag;
    private boolean lockOn;

    private VisionStatus visionStatus;

    private CommandXboxController controller;

    public ControllerInput(CommandXboxController controller) {
        this.controller = controller;
        this.visionStatus = VisionStatus.NONE;
    }

    @Override
    public void periodic() {
        // controls the X and Y directions of the robot respectively
        x = controller.getLeftX();
        y = controller.getLeftY();

        // simple deadzone, we can change this to be a circle instead of a square but it doesn't really matter
        if (Math.abs(x) < 0.05 && Math.abs(y) < 0.05) {
            x = 0;
            y = 0;
        }

        // this controls the robot spinning 
        theta = controller.getRightX();

        if (Math.abs(theta) < 0.05) {
            theta = 0;
        }

        // NOS :)
        nos = controller.getRightTriggerAxis() > 0.75;

        // field relative :)
        //fieldRelative = !controller.getRightBumperButton();

        // This is just a basic thing - we can make it more complex if we want for auto or smth
        //alignWithTag = controller.getLeftBumperButton();

        if (alignWithTag) visionStatus = VisionStatus.ALIGN_TAG;
        else if (lockOn) visionStatus = VisionStatus.LOCKON;
        else visionStatus = VisionStatus.NONE;
    }

    /**
     * Uses controller input to return a ChassisSpeeds object.

     * @return chassisSpeeds - the spped of the robot calclated by the controller
     */
    public ChassisSpeeds controllerChassisSpeeds(PIDController turnPID, double currentAngle) {
        double turnSpeed = 0;
        if (Math.abs(theta) < 0.01) {
            double error = currentAngle;
            turnPID.setSetpoint(0);
            if (Math.abs(error) > 2) turnSpeed = turnPID.calculate(error);
            turnSpeed = 0;
        } else  {
            turnSpeed = theta * 2;
        }

        ChassisSpeeds chassisSpeeds;

        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                DriverConstants.highDriveSpeed * y,
                -DriverConstants.highDriveSpeed * x,
                turnSpeed,
                Rotation2d.fromDegrees(currentAngle)
            );
        } else {
            // If we are not in field relative mode, we are in robot relative mode
            chassisSpeeds = new ChassisSpeeds(
                DriverConstants.highDriveSpeed * y,
                -DriverConstants.highDriveSpeed * x,
                turnSpeed
            );
        }

        return chassisSpeeds;
    }

    public Command toggleNos = Commands.runOnce(() -> {
        nos = !nos;
    });

    public Command toggleFeildRelative = Commands.runOnce(() -> {
        fieldRelative = !fieldRelative;
    });

    public Command toggleAlignTag = Commands.runOnce(() -> {
        alignWithTag = !alignWithTag;
    });

    public Command toggleLockOn = Commands.runOnce(() -> {
        lockOn = !lockOn;
    });


    public boolean nos() {return nos;}
    public VisionStatus visionStatus() {return visionStatus;}
}
