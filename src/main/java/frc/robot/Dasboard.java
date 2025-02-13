package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import choreo.auto.AutoChooser;



public class Dasboard {
    
    public Dasboard(Swerve Swerve, AutoChooser autoChooser) {
        SmartDashboard.putBoolean("test", false);
        SmartDashboard.putNumber("Heading", Swerve.getPose().getRotation().getDegrees());
        double[] acceleration = Swerve.getAcceleration();
        SmartDashboard.putNumber("AccelX", acceleration[0]);
        SmartDashboard.putNumber("AccelY", acceleration[1]);
        SmartDashboard.putNumber("AccelZ", acceleration[2]);
        SmartDashboard.putData(autoChooser);
    }
}
