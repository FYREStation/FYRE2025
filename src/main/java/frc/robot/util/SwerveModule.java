package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriverConstants;

/** A wrapper class to encapsulate a swerve module. */
public class SwerveModule {

    // the index of this swerve module
    private final int index;
    
    private final SparkMax swerveMotor;
    private final SparkMaxConfig swerveConfig;
    private final RelativeEncoder swerveEncoder;

    private final SparkMax driveMotor;
    private final SparkMaxConfig driveConfig;
    private final RelativeEncoder driveEncoder;

    private final DutyCycleEncoder swerveEncoderAbsolute;

    private final SparkClosedLoopController swervePID;

    double lastMotorSpeed;
    double lastMotorSetTime;

    /** Represents a speed and angle for a swerve module. */
    public class SwerveAngleSpeed {
        double targetAngle;
        int multiplier;
    }

    /**
     * Initializes a new swerve module with the given index.

     * @param index - the index of the module in order of the defined placement
     */
    public SwerveModule(int index) {

        this.index = index;

        swerveMotor = new SparkMax(
            DriverConstants.swerveMotorPorts[index], 
            SparkLowLevel.MotorType.kBrushless
        );
        swerveEncoder = swerveMotor.getEncoder();
        swerveEncoderAbsolute = new DutyCycleEncoder(DriverConstants.encoders[index]);
        swerveConfig = new SparkMaxConfig();

        driveMotor = new SparkMax(
            DriverConstants.driveMotorPorts[index],
            SparkLowLevel.MotorType.kBrushless
        );
        driveEncoder = driveMotor.getEncoder();
        driveConfig = new SparkMaxConfig();

        swervePID = swerveMotor.getClosedLoopController();


        // configure the swerve motor
        swerveConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(15);

        swerveConfig.encoder
            .positionConversionFactor(360 / 12.8)
            .velocityConversionFactor(360 / 12.8);

        swerveConfig.signals
            .primaryEncoderPositionPeriodMs(20);

        swerveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                DriverConstants.swerveP,
                DriverConstants.swerveI,
                DriverConstants.swerveD,
                DriverConstants.swerveFF
            ).iZone(0)
            .outputRange(-1, 1);

        // configure the drive motor
        driveConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);

        driveConfig.encoder
            .positionConversionFactor(1 / 8.14)
            .velocityConversionFactor(1 / 8.14);

        driveConfig.signals
            .primaryEncoderPositionPeriodMs(50);


        swerveMotor.configure(
            swerveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        driveMotor.configure(
            driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        double relativeZero = getAbsolutePosition();

        REVLibError error = swerveEncoder.setPosition(relativeZero);

        swervePID.setReference(
            DriverConstants.absoluteOffsets[index],
            SparkMax.ControlType.kPosition
        );
        
        if (error.equals(REVLibError.kOk)) System.out.println("Swerve Module " + index + " is initialized!");
    }

    public void setSwerveReference(double value) {
        swervePID.setReference(value, ControlType.kPosition);
    }


    public void setSwerveEncoder(double position) {
        swerveEncoder.setPosition(position);
    }

    public double getSwervePosition() {
        return swerveEncoder.getPosition();
    }

    public void setDriveEncoder(double position) {
        driveEncoder.setPosition(position);
    }

    public double getDrivePostion() {
        return driveEncoder.getPosition();
    }

    public void printModuleStatus() {
        System.out.printf("%d: %f\n", index, getAbsolutePosition());
    }

    public double getAbsolutePosition() {
        return (swerveEncoderAbsolute.get() * 360);
    }

    /**
     * Returns a SwerveModuleState object from this module.

     * @return swerveModuleState - the object from this module
     */
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity() * DriverConstants.metersPerRotation,
            Rotation2d.fromDegrees(swerveEncoder.getPosition())
        );
    }

    /**
     * Returns a SwerveModulePosition object from this module.

     * @return swerveModulePosition - the object from this module
     */
    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition() * DriverConstants.metersPerRotation,
            Rotation2d.fromDegrees(swerveEncoder.getPosition())
        );
    }

    public boolean setupCheck() {
        return Math.abs(swerveEncoder.getPosition() - DriverConstants.absoluteOffsets[index]) > 1.5;
    }

    /**
     * Returns the absolute angle this module needs to approach.

     * @param targetAngle - the angle the module should be at
     * @param currentAngle - the current position of the module
     * @return absoluteTarget - the absolute angle the target should approach
     */
    public SwerveAngleSpeed getAbsoluteTarget(double targetAngle, double currentAngle) {
        
        double angleDiff = targetAngle - doubleMod(doubleMod(currentAngle, 360) + 360, 360);

        if (angleDiff > 180) {
            angleDiff -= 360;
        } else if (angleDiff < -180) {
            angleDiff += 360;
        }

        SwerveAngleSpeed speed = new SwerveAngleSpeed();
        speed.targetAngle = currentAngle + angleDiff;
        speed.multiplier = 1;

        return speed;

        // targetAngle += 180;
        // int multiplier = 1;

        // double angleDiff = targetAngle - doubleMod(doubleMod(currentAngle, 360) + 360, 360);

        // if (angleDiff > 180) {
        //     angleDiff -= 360;
        // } else if (angleDiff < -180) {
        //     angleDiff += 360;
        // }

        // if (angleDiff < -90){
        //     angleDiff += 180;
        //     multiplier = -1;
        // } else if (angleDiff > 90){
        //     angleDiff -= 180;
        //     multiplier = -1;
        // }

        // SwerveAngleSpeed absoluteTarget = new SwerveAngleSpeed();
        // absoluteTarget.multiplier = multiplier;
        // absoluteTarget.targetAngle = currentAngle + angleDiff;
        
        // return absoluteTarget;
    }

    /**
     * Drives this module with the given module state.

     * @param moduleState - the module state for this module to use
     * @param rotate - whether or not we need to try to rotate
     * @param nos - if NOS (high drive mode) is enabled
     */
    public void driveModule(SwerveModuleState moduleState, boolean rotate, boolean nos) {
        double currentAngle = swerveEncoder.getPosition();
        double targetAngle = moduleState.angle.getDegrees();

        SwerveAngleSpeed absoluteTarget = getAbsoluteTarget(targetAngle, currentAngle);

        if (rotate) {
            swervePID.setReference(absoluteTarget.targetAngle, SparkMax.ControlType.kPosition);
        }

        setMotorSpeed(
            absoluteTarget.multiplier
            * moduleState.speedMetersPerSecond
            * DriverConstants.speedModifier
            * (nos ? 2.25 : 1)
        );
    }
    
    
    /**
     * Sets the velocity of the drive motor using feedforward.

     * @param velocity - the target speed to set the module of
     */
    private void setMotorSpeed(double velocity) {
        double time = Timer.getFPGATimestamp();
        double acceleration = time - lastMotorSetTime > 0.1
            ? 0
            : (velocity - lastMotorSpeed) / (time - lastMotorSetTime);

        double ffv = DriverConstants.driveFeedForward[index].calculateWithVelocities(velocity, 0);
        driveMotor.setVoltage(ffv);
        lastMotorSpeed = velocity;
        lastMotorSetTime = time;
    }


    private double doubleMod(double x, double y) {
        // x mod y behaving the same way as Math.floorMod but with doubles
        return (x - Math.floor(x / y) * y);
    }
}
