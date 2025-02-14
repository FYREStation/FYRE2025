package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import choreo.auto.AutoChooser;



public class Dasboard {
    
    public Dasboard(Swerve swerve, Vision vision, AutoChooser autoChooser,int camIndex) {
        SmartDashboard.putBoolean("test", false);
        SmartDashboard.putBoolean("Lockon", vision.getNumTags(camIndex) > 0);
        SmartDashboard.putNumber("Heading", swerve.getPose().getRotation().getDegrees());
        double[] acceleration = swerve.getAcceleration();
        SmartDashboard.putNumber("AccelX", acceleration[0]);
        SmartDashboard.putNumber("AccelY", acceleration[1]);
        SmartDashboard.putNumber("AccelZ", acceleration[2]);
        SmartDashboard.putData(autoChooser);
    }
}
