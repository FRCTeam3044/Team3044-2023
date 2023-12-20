package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.packages.HDrive.HDriveKinematics;
import frc.packages.HDrive.HDrivePoseEstimator;
import frc.packages.HDrive.HDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.packages.HDrive.HDrivetrainDrive;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.supers.HDriveSuper;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import java.util.Optional;

public class HDrive extends HDriveSuper {
    public double kLeftEncoderDistancePerPulse = 0.0;
    public double kRightEncoderDistancePerPulse = 0.0;
    public double kLateralEncoderDistancePerPulse = 0.0;

    public void tankDrive(double leftSpeed, double rightSpeed, double lateralSpeed) {
        drive.tankDrive(leftSpeed, rightSpeed, lateralSpeed);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts, double lateralVolts) {
        drive.tankDrive(leftVolts / 12, rightVolts / 12, lateralVolts / 12);
    }

    public double getLeftEncoderDistance() {
        return getLeftEncoder().getPosition() / kLeftEncoderDistancePerPulse;
    }

    public double getRightEncoderDistance() {
        return getRightEncoder().getPosition() / kRightEncoderDistancePerPulse;
    }

    public double getLateralEncoderDistance() {
        return getLateralEncoder().getPosition() / kLateralEncoderDistancePerPulse;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    // The left-side drive encoder
    private final RelativeEncoder leftEncoder;

    // The right-side drive encoder
    private final RelativeEncoder rightEncoder;

    // The lateral drive encoder
    private final RelativeEncoder lateralEncoder;

    public AHRS NavX = new AHRS();

    HDriveKinematics HDriveKinematics;

    private PhotonCameraWrapper1 photonPoseEstimator1;
    private PhotonCameraWrapper2 photonPoseEstimator2;

    /**
     * Creates a new DriveSubsystem.
     */
    public HDrive() {
        System.out.println("Starting HDrive Real");
        // The motors on the left side of the drive.
        leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
        leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);

        // The motors on the right side of the drive.
        rightMotor1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);
        rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);

        // The lateral movement motors of H Drive
        lateralMotor1 = new CANSparkMax(DriveConstants.kLateralMotor1Port, MotorType.kBrushless);
        lateralMotor2 = new CANSparkMax(DriveConstants.kLateralMotor2Port, MotorType.kBrushless);
        lateralMotors = new MotorControllerGroup(lateralMotor1, lateralMotor2);

        drive = new HDrivetrainDrive(leftMotors, rightMotors, lateralMotors);
        drive.setSafetyEnabled(false);

        //Solenoids for lateral motors
        lateralSolenoid1 = RobotContainer.m_pH.makeDoubleSolenoid(DriveConstants.kForwardLateral1Port, DriveConstants.kReverseLateral1Port);
        lateralSolenoid2 = RobotContainer.m_pH.makeDoubleSolenoid(DriveConstants.kForwardLateral2Port, DriveConstants.kReverseLateral2Port);

        up = false;

        gyro = new ADXRS450_Gyro();

        //vision
        photonPoseEstimator1 = new PhotonCameraWrapper1();
        photonPoseEstimator2 = new PhotonCameraWrapper2();


        rightMotor1.restoreFactoryDefaults();
        rightMotor2.restoreFactoryDefaults();
        leftMotor1.restoreFactoryDefaults();
        leftMotor2.restoreFactoryDefaults();
        lateralMotor1.restoreFactoryDefaults();
        lateralMotor2.restoreFactoryDefaults();

        leftEncoder = leftMotor1.getEncoder();
        rightEncoder = rightMotor1.getEncoder();
        lateralEncoder = lateralMotor1.getEncoder();

        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);
        lateralMotor2.follow(lateralMotor1);

        lateralMotor1.setInverted(false);
        lateralMotor2.setInverted(false);

        leftMotor1.setInverted(true);
        leftMotor2.setInverted(true);

        leftMotor1.setSmartCurrentLimit(40);
        leftMotor2.setSmartCurrentLimit(40);
        rightMotor1.setSmartCurrentLimit(40);
        rightMotor2.setSmartCurrentLimit(40);
        lateralMotor1.setSmartCurrentLimit(40);
        lateralMotor2.setSmartCurrentLimit(40);

        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.

        // Sets the distance per pulse for the encoders

        resetEncoders();
        System.out.println("HDrive Created (REAL)");
        System.out.println("Left M1 Port " + DriveConstants.kLeftMotor1Port);
        System.out.println("Left M2 Port " + DriveConstants.kLeftMotor2Port);
        System.out.println("Right M1 Port " + DriveConstants.kRightMotor1Port);
        System.out.println("Right M2 Port " + DriveConstants.kRightMotor2Port);
        System.out.println("Lateral M1 Port " + DriveConstants.kLateralMotor1Port);
        System.out.println("Lateral M2 Port " + DriveConstants.kLateralMotor2Port);

        System.out.println("Left M1 Inverted? " + leftMotor1.getInverted());
        System.out.println("Left M2 Inverted? " + leftMotor2.getInverted());
        System.out.println("Right M1 Inverted? " + rightMotor1.getInverted());
        System.out.println("Right M2 Inverted? " + rightMotor2.getInverted());
        System.out.println("Lateral M1 Inverted? " + lateralMotor1.getInverted());
        System.out.println("Lateral M2 Inverted? " + lateralMotor2.getInverted());
        HDriveKinematics = new HDriveKinematics(DriveConstants.kTrackwidthMeters);

        engageLateralDrive(true);

        poseEstimator = new HDrivePoseEstimator(
            HDriveKinematics,
            DriveConstants.kTrackwidthMeters,
            /*Rotation2d.fromDegrees(NavX.getYaw())*/Rotation2d.fromDegrees(10),
            0, 0, 0,
            new Pose2d(),
            VecBuilder.fill(0.03, 0.03, 0.01),
            VecBuilder.fill(0, 0, 0)
        );

    }

    int iter = 0;
    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Temperatures/Lateral 1", lateralMotor1.getMotorTemperature());
        Logger.getInstance().recordOutput("Temperatures/Lateral 2", lateralMotor2.getMotorTemperature());
        Logger.getInstance().recordOutput("Temperatures/Left 1", leftMotor1.getMotorTemperature());
        Logger.getInstance().recordOutput("Temperatures/Left 2", leftMotor2.getMotorTemperature());
        Logger.getInstance().recordOutput("Temperatures/Right 1", rightMotor1.getMotorTemperature());
        Logger.getInstance().recordOutput("Temperatures/Right 2", rightMotor2.getMotorTemperature());

        poseEstimator.update(Rotation2d.fromDegrees(-NavX.getYaw()),
            -leftEncoder.getPosition() * (Math.PI * Constants.DriveConstants.leftWheelRadius) / Constants.GearboxConstants.kDriveRatio,
            -rightEncoder.getPosition() * (Math.PI * Constants.DriveConstants.rightWheelRadius) / Constants.GearboxConstants.kDriveRatio,
            lateralEncoder.getPosition() * (Math.PI * Constants.DriveConstants.lateralWheelRadius) / Constants.GearboxConstants.kLateralRatio);

        SmartDashboard.putNumber("Debug/NavX", -NavX.getYaw());
        SmartDashboard.putNumber("Debug/Left", leftEncoder.getPosition() * (Math.PI * Constants.DriveConstants.leftWheelRadius) / Constants.GearboxConstants.kDriveRatio);
        SmartDashboard.putNumber("Debug/Right", rightEncoder.getPosition() * (Math.PI * Constants.DriveConstants.rightWheelRadius) / Constants.GearboxConstants.kDriveRatio);
        SmartDashboard.putNumber("Debug/Lateral", lateralEncoder.getPosition() * (Math.PI * Constants.DriveConstants.lateralWheelRadius) / Constants.GearboxConstants.kLateralRatio);
        SmartDashboard.putNumber("Debug/Vision Uncertainty", logScaler(getPose().getX()));
        SmartDashboard.putNumber("hdrive periodic", iter);
// Decreases weighting of vision measurements the further away from AprilTags the robot is
        double PoseX = getPose().getX();
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(logScaler(PoseX),
            logScaler(PoseX),
            logScaler(PoseX)));

        Optional<EstimatedRobotPose> result1 = photonPoseEstimator1.getEstimatedGlobalPose1(poseEstimator.getEstimatedPosition());
        Optional<EstimatedRobotPose> result2 = photonPoseEstimator2.getEstimatedGlobalPose2(poseEstimator.getEstimatedPosition());

        if (result1.isPresent()) {
            SmartDashboard.putNumber("result1", iter);
            EstimatedRobotPose camPose = result1.get();
            Pose2d pose = camPose.estimatedPose.toPose2d();
            double[] poseArray = {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
            SmartDashboard.putNumberArray("Result 1 Photon Pose", poseArray);
            poseEstimator.addVisionMeasurement(
                pose,
                camPose.timestampSeconds
            );
        }
        if(result2.isPresent()) {
            SmartDashboard.putNumber("result2", iter);
            EstimatedRobotPose camPose = result2.get();

            poseEstimator.addVisionMeasurement(
                camPose.estimatedPose.toPose2d(),
                camPose.timestampSeconds
            );
        }

        double[] pose = {poseEstimator.getEstimatedPosition().getX(), poseEstimator.getEstimatedPosition().getY(), poseEstimator.getEstimatedPosition().getRotation().getDegrees()};

        iter++;
        SmartDashboard.putNumberArray("Robot Pose", pose);
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public HDriveWheelSpeeds getWheelSpeeds() {
        return new HDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity(), lateralEncoder.getVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        poseEstimator.resetPosition(Rotation2d.fromDegrees(-NavX.getYaw()),
            -leftEncoder.getPosition() * (2 * Math.PI * Constants.DriveConstants.leftWheelRadius) / Constants.GearboxConstants.kDriveRatio,
            -rightEncoder.getPosition() * (2 * Math.PI * Constants.DriveConstants.rightWheelRadius) / Constants.GearboxConstants.kDriveRatio,
            lateralEncoder.getPosition() * (2 * Math.PI * Constants.DriveConstants.lateralWheelRadius) / Constants.GearboxConstants.kLateralRatio,
            pose);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot, double lateral) {
        drive.arcadeDrive(fwd, rot, lateral);
    }

    /**
     * Drives the robot using tank controls.
     *
     * @param right the commanded forward movement
     * @param left  the commanded rotation
     */
    public void tankDrive(double right, double left) {
        drive.tankDrive(right, left, 0);
    }

    /**
     * Controls the left, right, & lateral of the drive directly with voltages.
     *
     * @param leftVolts    the commanded left output
     * @param rightVolts   the commanded right output
     * @param lateralVolts the commanded lateral output
     */
    public void driveVolts(double leftVolts, double rightVolts, double lateralVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        lateralMotors.setVoltage(lateralVolts);
        drive.feed();
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     *                   with lateral volts set to zero this is to support a ramsete
     *                   controller
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        // print out left and right volts
        tankDrive(leftVolts / 12, rightVolts / 12);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        lateralEncoder.setPosition(0);
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

    /**
     * Gets the lateral drive encoder.
     *
     * @return the lateral drive encoder
     */
    public RelativeEncoder getLateralEncoder() {
        return lateralEncoder;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        NavX.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return NavX.getYaw();
    }

    /**
     * Sets the lateral drive plant status by moving the solenoids.
     */
    public void engageLateralDrive(boolean engage) {
        if (engage) {
            lateralSolenoid1.set(Value.kReverse);
            lateralSolenoid2.set(Value.kReverse);
        } else {
            lateralSolenoid1.set(Value.kForward);
            lateralSolenoid2.set(Value.kForward);
        }
    }

    /**
     * Toggles the lateral drive plant status swapping its state.
     */
    public void toggleLateralDrive() {
        up = !up;
        engageLateralDrive(up);
    }
}