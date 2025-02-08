/*
Copyright (c) FIRST and other WPILib contributors.
Open Source Software; you can modify and share it under the terms of
the WPILib BSD license file in the root directory of this project.
*/

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /** A set of constants related to the drivetrain. */
    public static class DriveConstants {

        public static final int frontLeftSwervePort = 1;
        public static final int frontRightSwervePort = 3;
        public static final int backLeftSwervePort = 5;
        public static final int backRightSwervePort = 7;

        public static final int[] swerveMotorPorts = {
            frontLeftSwervePort,
            frontRightSwervePort,
            backLeftSwervePort,
            backRightSwervePort
        };

        public static final int frontLeftDrivePort = 2;
        public static final int frontRightDrivePort = 4;
        public static final int backLeftDrivePort = 6;
        public static final int backRightDrivePort = 8;

        public static final int[] driveMotorPorts = {
            frontLeftDrivePort,
            frontRightDrivePort,
            backLeftDrivePort,
            backRightDrivePort
        };

        public static final int frontLeftEncoder = 0;
        public static final int frontRightEncoder = 1;
        public static final int backLeftEncoder = 3;
        public static final int backRightEncoder = 4;

        public static final int[] encoders = {
            frontLeftEncoder,
            frontRightEncoder,
            backLeftEncoder,
            backRightEncoder
        };

        public static final double[] absoluteOffsets = {
            40.5,
            256.0,
            11.5,
            344.5
        };

        public static final double metersFromRobotCenter = 0.27305;

        public static final Translation2d frontLeft = new Translation2d(
            metersFromRobotCenter, metersFromRobotCenter);
        public static final Translation2d frontRight = new Translation2d(
            metersFromRobotCenter, -metersFromRobotCenter);
        public static final Translation2d backLeft = new Translation2d(
            -metersFromRobotCenter, metersFromRobotCenter);
        public static final Translation2d backRight = new Translation2d(
            -metersFromRobotCenter, -metersFromRobotCenter);

        public static final double swerveP = 0.032;
        public static final double swerveI = 0.0;
        public static final double swerveD = 0.015;
        public static final double swerveFF = 0;

        public static final double xyP = 10;
        public static final double xyI = 10;
        public static final double xyD = 10;

        public static final double turnP = 0.053;
        public static final double turnI = 0.0;
        public static final double turnD = 0.0;
        public static final double turnR = 0.02;

        public static final double driveKs = 0.18;
        public static final double driveKv = 3.12;
        public static final double driveKa = 0.18;

        public static final SimpleMotorFeedforward[] driveFeedForward = {
            new SimpleMotorFeedforward(driveKs, driveKv, driveKa),
            new SimpleMotorFeedforward(driveKs, driveKv, driveKa),
            new SimpleMotorFeedforward(driveKs, driveKv, driveKa),
            new SimpleMotorFeedforward(driveKs, driveKv, driveKa)
        };


        public static final double highDriveSpeed = 1;//7.26;
        public static final double speedModifier = 0.75;

        public static final double inchesPerRotation = Math.PI * 3.875;
        public static final double metersPerRotation = Units.inchesToMeters(inchesPerRotation);

        public static final double swerveRotationToDegrees = 360 / 12.8;
        public static final double driveMotorToWheel = 1 / 8.14;

        public static final double nosBooster = 2.25;
    }

    /** A set of constants relating to the elevator. */
    public static class ElevatorLiftConstants {
        public static final int elevatorMotor1Port = -1234567890;
        public static final int elevatorMotor2Port = -123456789;
        public static final double maxRotations = -1234567890;
        public static final double staticGain = -1234567890;
        public static final double gravityGain = -1234567890;
        public static final double velocityGain = -1234567890;
        public static final int bottomLimitSwitchPort = -1234567890;
        public static final int topLimitSwitchPort = -1234567890;
        public static final double kP = -1234567890;
        public static final double kI = -1234567890;
        public static final double kD = -1234567890;
        public static final double maxAcceleration = -1234567890;
        public static final double maxVelocity = -1234567890;
        public static final double elvevatorThrottle = -1234567890;
        public static final double leadercanID = -1234567890;

    }

    /** A set of constants relating to the intake. */
    public static class IntakeConstants {
        public static final int intakeWheelPort = -1234567890;
        public static final int intakeActuationPort = 12;
        public static final double rotationsToBottom = -1234567890;
        public static final int intakeEncoderA = -1234567890;
    }

    /** A set of constants relating to the arm. */
    public static class ArmConstants {
        public static final int armPort = -1234567890;
        public static final double staticGain = -1234567890;
        public static final double gravityGain = -1234567890;
        public static final double velocityGain = -1234567890;
        public static final double kP = -1234567890;
        public static final double kI = -1234567890;
        public static final double kD = -1234567890;
        public static final double maxRotations = -1234567890;
        public static final double maxVelocity = -1234567890;
        public static final double maxAcceleration = -1234567890;

    }

    /** A set of constants relating to the claw. */
    public static class ClawConstants {
        public static final int clawMotorPort = -1234567890;
        public static final int clawWheelMotorPort = -1234567890;
        public static final double rotationsToBottom = -1234567890;
    }

    /** A set of constants relating to the climber. */
    public static class ClimberConstants {
        public static final int climberMotorPort = -1234567890;
        public static final double rotationsToBottom = -1234567890;
    }

    /** A set of constants relating to operator controls. */
    public static class OperatorConstants {
        public static final int driverControllerPort = 0;
    }

    /** A set of constants relating to vision. */
    public static class VisionConstants {
        public static final String ipAddress = "ws://10.42.0.123";
        public static final int[] CameraRotations = {0};
        public static HashMap<String, Integer> apriltagAngles = new HashMap<>();
        
        /** Constructs apriltags angles hashmap. */
        public VisionConstants() {
            apriltagAngles.put("13", 0);
            apriltagAngles.put("tag2", 45);
            apriltagAngles.put("tag3", 60);
            apriltagAngles.put("tag4", 90);
        }
    }

}