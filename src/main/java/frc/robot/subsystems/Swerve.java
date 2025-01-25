package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

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
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriverConstants;
import frc.robot.util.ControllerInput;
import frc.robot.util.ControllerInput.VisionStatus;

public class Swerve extends SubsystemBase{

    private final ControllerInput controllerInput;

    private final Vision visionSystem; 
    private final AHRS gyroAhrs;

    private final SparkMax[] swerveMotors = new SparkMax[4];
    private final SparkMaxConfig[] swerveConfig = new SparkMaxConfig[4];

    private final SparkMax[] driveMotors = new SparkMax[4];
    private final SparkMaxConfig[] driveConfig = new SparkMaxConfig[4];

    private final RelativeEncoder[] swerveEncoders = new RelativeEncoder[4];

    private final DutyCycleEncoder[] swerveEncodersAbsolute = new DutyCycleEncoder[4];

    private final SparkClosedLoopController[] swervePID = new SparkClosedLoopController[4];

    private final SwerveDrivePoseEstimator poseEstimator;

    private Pose2d currentPose;

    private final PIDController turnPID = new PIDController(
        7.52,
        0.01,
        0.00,
        0.02
    );

    private final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        DriverConstants.frontLeft,
        DriverConstants.frontRight,
        DriverConstants.backLeft,
        DriverConstants.backRight
    );

    double lastMotorSpeeds[] = new double[4];
    double lastMotorSetTimes[] = new double[4];

    private double turnTarget = 0;

    private double[] initialStates = new double[4];

    boolean setupComplete = false;

    public class SwerveAngleSpeed {
        double targetAngle;
        int multiplier;
    }

    public Swerve(ControllerInput controller, Vision visionSystem) {

        // assign constructor variables
        this.controllerInput = controller;
        this.visionSystem = visionSystem;

        this.currentPose = new Pose2d(2.55, 5.1, new Rotation2d(0));

        // define the gyro
        gyroAhrs = new AHRS(NavXComType.kMXP_SPI);
        // reset the gyro
        gyroAhrs.reset();

        // sets up the motors
        setupMotors();
        
        poseEstimator = new SwerveDrivePoseEstimator(
            swerveDriveKinematics,
            gyroAhrs.getRotation2d(),
            getSwerveModulePositions(),
            currentPose 
        );

    }

    @Override
    public void periodic() {

        // I don't know if this step is nessecary so look at this while testing :D
        currentPose = poseEstimator.update(Rotation2d.fromDegrees(gyroAhrs.getAngle()), getSwerveModulePositions());
        //currentPose = poseEstimator.getEstimatedPosition();

        if (setupComplete) {
            //swerveDrive(chooseDriveMode());
        } else setupCheck();
    }

    private ChassisSpeeds chooseDriveMode() {

        VisionStatus status = controllerInput.visionStatus();
        ChassisSpeeds speeds;

        switch (status) {
            case ALIGN_TAG: // lines the robot up with the tag
                speeds = visionSystem.alignTagSpeeds(0, null); 
                break;
            case LOCKON: // allows the robot to move freely by user input but remains facing the tag
                ChassisSpeeds controllerSpeeds = controllerChassisSpeeds();
                ChassisSpeeds lockonSpeeds = visionSystem.lockonTagSpeeds(0, null);
                speeds = new ChassisSpeeds(
                    controllerSpeeds.vxMetersPerSecond,
                    controllerSpeeds.vyMetersPerSecond,
                    lockonSpeeds.omegaRadiansPerSecond
                );
                break;
            default: // if all else fails - revert to drive controls
                speeds = controllerChassisSpeeds();
                break;
        }

        return speeds;
    }

    private void setupCheck() {
        visionSystem.clear();
        for (int i = 0; i < 4; i++) {
            if (Math.abs(swerveEncoders[i].getPosition() - DriverConstants.absoluteOffsets[i]) > 1.5) {
                return;
            }
        }
        setupComplete = true;
        resetEncoders();
        for (int i = 0; i < 4; i++) swervePID[i].setReference(0, ControlType.kPosition);
        try {TimeUnit.MILLISECONDS.sleep(20);} catch (InterruptedException e) {e.getStackTrace();}
    }

    private void setupMotors() {
        System.out.println("setting up motors");

        // if this needs to loop more than 4 times, something is very wrong
        for (int i = 0; i < 4; i++) {

            swerveMotors[i] = new SparkMax(
                DriverConstants.swerveMotorPorts[i],
                SparkLowLevel.MotorType.kBrushless
            );

            driveMotors[i] = new SparkMax(
                DriverConstants.driveMotorPorts[i],
                SparkLowLevel.MotorType.kBrushless
            );
            
            swerveEncoders[i] = swerveMotors[i].getEncoder();
            swerveEncodersAbsolute[i] = new DutyCycleEncoder(Constants.DriverConstants.encoders[i]);

            swerveConfig[i] = new SparkMaxConfig();
            driveConfig[i] = new SparkMaxConfig();

            swervePID[i] = swerveMotors[i].getClosedLoopController();

            swerveConfig[i]
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(15);

            driveConfig[i]
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30);


            swerveConfig[i].encoder
                .positionConversionFactor(360 / 12.8)
                .velocityConversionFactor(360 / 12.8);

            driveConfig[i].encoder
                .positionConversionFactor(1/8.14)
                .velocityConversionFactor(1/8.14);


            swerveConfig[i].closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(
                    DriverConstants.swerveP,
                    DriverConstants.swerveI,
                    DriverConstants.swerveD,
                    DriverConstants.swerveFF
                )
                .iZone(0)
                .outputRange(-1, 1);

            swerveConfig[i].signals
                .primaryEncoderPositionPeriodMs(20);

            driveConfig[i].signals
                .primaryEncoderPositionPeriodMs(100);

            // save config into the sparks
            swerveMotors[i].configure(swerveConfig[i], ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            driveMotors[i].configure(driveConfig[i], ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


            double relativeZero = getAbsolutePosition(i);

            REVLibError error = swerveEncoders[i].setPosition(relativeZero);

            // set the swerve pid to try to reset to zero
            swervePID[i].setReference(
                DriverConstants.absoluteOffsets[i],
                SparkMax.ControlType.kPosition
            );

            if (error.equals(REVLibError.kOk)) System.out.println("Motor controller took value");

            initialStates[i] = relativeZero;

        }

        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        turnPID.setSetpoint(0);
    }

    public SwerveDriveKinematics getSwerveDriveKinematics() {return swerveDriveKinematics;}

    public ChassisSpeeds getRobotState() {return swerveDriveKinematics.toChassisSpeeds(getSwerveModuleStates());}

    public double getAngle() {return gyroAhrs.getAngle();}

    public Pose2d getPose() {return currentPose;} 

    public void resetGyro() {
        gyroAhrs.reset();
        turnTarget = 0;
    }

    public void resetEncoders() {
        for (int i = 0; i < 4; i++) {
            swerveEncoders[i].setPosition(0);
        }
    }

    public void printModuleStatus() {
        for (int i = 0; i < 4; i++) {
            System.out.println(i + ": " + swerveEncoders[i].getPosition());
        }
    }


    public void resetOdometry(Pose2d pose) {
        //resetEncoders();

        //gyroAhrs.reset();
        //gyroAhrs.setAngleAdjustment(pose.getRotation().getDegrees());

        currentPose = pose;
        poseEstimator.resetPose(pose);

    }

    double doubleMod(double x, double y) {
        // x mod y behaving the same way as Math.floorMod but with doubles
        return (x - Math.floor(x / y) * y);
    }

    public double getAbsolutePosition(int moduleNumber) {
        return 360 - (swerveEncodersAbsolute[moduleNumber].get() * 360);
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] swerveModuleState = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            SwerveModuleState moduleState = new SwerveModuleState(
                driveMotors[i].getEncoder().getVelocity() * DriverConstants.metersPerRotation,
                Rotation2d.fromDegrees(getAbsolutePosition(i))
            );
            swerveModuleState[i] = moduleState;
        }

        return swerveModuleState;
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            SwerveModulePosition modulePosition = new SwerveModulePosition(
                driveMotors[i].getEncoder().getPosition() * DriverConstants.metersPerRotation,
                Rotation2d.fromDegrees(swerveEncoders[i].getPosition())
            );
            swerveModulePositions[i] = modulePosition;
        }

        return swerveModulePositions;
    }

    public ChassisSpeeds controllerChassisSpeeds() {
        double turnSpeed = 0;
        if (Math.abs(controllerInput.theta()) < 0.01) {
            double error = turnTarget + getAngle();
            turnPID.setSetpoint(0);
            if (Math.abs(error) > 2) turnSpeed = turnPID.calculate(error);
            turnSpeed = 0;
        } else  {
            turnSpeed = controllerInput.theta(); // code orange multiplies this by 6
            turnTarget = getAngle();
        }

        ChassisSpeeds chassisSpeeds;

        if (controllerInput.fieldRelative()){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                DriverConstants.highDriveSpeed * controllerInput.x(),
                DriverConstants.highDriveSpeed * controllerInput.y(),
                turnSpeed,
                Rotation2d.fromDegrees(getAngle())
            );
        } else {
            // If we are not in field relative mode, we are in robot relative mode, so dont do the field thing
            chassisSpeeds = new ChassisSpeeds(
                DriverConstants.highDriveSpeed * controllerInput.x(),
                DriverConstants.highDriveSpeed * controllerInput.y(),
                turnSpeed
            );
        }

        return chassisSpeeds;
    }

    public void swerveDrive(ChassisSpeeds chassisSpeeds) {

        SwerveModuleState[] moduleState = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        boolean rotate = chassisSpeeds.vxMetersPerSecond != 0 || chassisSpeeds.vyMetersPerSecond != 0 || chassisSpeeds.omegaRadiansPerSecond != 0;

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleState, DriverConstants.highDriveSpeed);

        for (int i = 0; i < 4; i++) {
            SwerveModuleState targetState = moduleState[i];
            double currentAngle = swerveEncoders[i].getPosition();
            double targetAngle = targetState.angle.getDegrees();
            SwerveAngleSpeed absoluteTarget = getAbsoluteTarget(targetAngle, currentAngle);

            if (rotate) {
                swervePID[i].setReference(absoluteTarget.targetAngle, SparkMax.ControlType.kPosition);
            }

            setMotorSpeed(i, absoluteTarget.multiplier * targetState.speedMetersPerSecond * DriverConstants.speedModifier);
            // driveMotors[i].set(
            //     controllerInput.getMagnitude() 
            //     * (controllerInput.nos() ? DriverConstants.highDriveSpeed : DriverConstants.standardDriveSpeed)
            //     * DriverConstants.speedModifier
            // );

        }
    }

    public void setMotorSpeed(int module, double velocity) {
        double time = Timer.getFPGATimestamp();
        double acceleration = time - lastMotorSetTimes[module] > 0.1
            ? 0
            : (velocity - lastMotorSpeeds[module]) / (time - lastMotorSetTimes[module]);

        double ffv = DriverConstants.driveFeedForward[module].calculateWithVelocities(velocity, acceleration);
        driveMotors[module].setVoltage(ffv);
        lastMotorSpeeds[module] = velocity;
        lastMotorSetTimes[module] = time;
    }

    /**
     * Returns the absolute angle a module needs to approach
     * @param targetAngle - the angle the module should be at
     * @param currentAngle - the current position of the module
     * @return absoluteTarget - the absolute angle the module needs to approach
     */
    private SwerveAngleSpeed getAbsoluteTarget(double targetAngle, double currentAngle) {

        //targetAngle += 180;

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

// =============== AUTO STUFF ==================== //

    private final PIDController xController = new PIDController(10, 0, 0);
    private final PIDController yController = new PIDController(10, 0, 0);

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            sample.vx, //+ xController.calculate(pose.getX(), sample.x),
            sample.vy, //+ yController.calculate(pose.getY(), sample.y),
            sample.omega,// + turnPID.calculate(pose.getRotation().getRadians(), sample.heading),
            Rotation2d.fromDegrees(getAngle())
        );

        swerveDrive(speeds);
    }
}