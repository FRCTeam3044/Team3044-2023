package frc.robot.subsystems_sim;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.packages.HDrive.HDriveKinematics;
import frc.packages.HDrive.HDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import frc.packages.HDrive.HDriveWheelSpeeds;
import frc.packages.HDrive.HDrivetrainDrive;
import frc.packages.HDrive.HDrivetrainSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.supers.HDriveSuper;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HDriveSim extends HDriveSuper {
    public void toggleLateralDrive() {
        up = !up;
        engageLateralDrive(up);
    }

    public void engageLateralDrive(boolean engage) {
        SmartDashboard.putBoolean("Lateral Engaged", engage);
    }

    // The left-side drive encoder
    private final Encoder leftEncoder = new Encoder(
        DriveConstants.kLeftEncoderPorts[0],
        DriveConstants.kLeftEncoderPorts[1],
        DriveConstants.kLeftEncoderReversed);

    // The right-side drive encoder
    private final Encoder rightEncoder = new Encoder(
        DriveConstants.kRightEncoderPorts[0],
        DriveConstants.kRightEncoderPorts[1],
        DriveConstants.kRightEncoderReversed);

    // The lateral drive encoder
    private final Encoder lateralEncoder = new Encoder(
        DriveConstants.kLateralEncoderPorts[0],
        DriveConstants.kLateralEncoderPorts[1],
        DriveConstants.kLateralEncoderReversed);

    // These classes help us simulate our drivetrain
    public static HDrivetrainSim drivetrainSimulator;
    private final EncoderSim leftEncoderSim;
    private final EncoderSim rightEncoderSim;
    private final EncoderSim lateralEncoderSim;
    // The Field2d class shows the field in the sim GUI
    private final Field2d fieldSim;
    private final ADXRS450_GyroSim gyroSim;

    HDriveKinematics HDriveKinematics;

    /**
     * Creates a new DriveSubsystem.
     */
    public HDriveSim() {
        System.out.println("Starting HDrive Sim");
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

        lateralSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, DriveConstants.kForwardLateral1Port, DriveConstants.kReverseLateral1Port);
        lateralSolenoid2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, DriveConstants.kForwardLateral2Port, DriveConstants.kReverseLateral2Port);

        gyro = new ADXRS450_Gyro();
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        rightMotors.setInverted(true);

        // Sets the distance per pulse for the encoders
        leftEncoder.setDistancePerPulse(DriveConstants.kSimEncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(DriveConstants.kSimEncoderDistancePerPulse);
        lateralEncoder.setDistancePerPulse(DriveConstants.kSimEncoderDistancePerPulse);

        resetEncoders();

        drivetrainSimulator = new HDrivetrainSim(
            DriveConstants.kDrivetrainPlant,
            DriveConstants.kLateralDrivetrainPlant,
            DriveConstants.kDriveGearbox,
            DriveConstants.kDriveGearing,
            DriveConstants.kTrackwidthMeters,
            DriveConstants.kWheelDiameterMeters / 2.0,
            0.05,
            VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005, 0.005, 0.005));

        HDriveKinematics = new HDriveKinematics(DriveConstants.kTrackwidthMeters);

        poseEstimator = new HDrivePoseEstimator(
            HDriveKinematics,
            DriveConstants.kTrackwidthMeters,
            Rotation2d.fromDegrees(gyro.getAngle()),
            getLeftEncoder().getDistance(),
            getRightEncoder().getDistance(),
            getLateralEncoder().getDistance(),
            getPose()
        );

        // The encoder and gyro angle sims let us set simulated sensor readings
        leftEncoderSim = new EncoderSim(leftEncoder);
        rightEncoderSim = new EncoderSim(rightEncoder);
        lateralEncoderSim = new EncoderSim(lateralEncoder);
        gyroSim = new ADXRS450_GyroSim(gyro);

        // the Field2d class lets us visualize our robot in the simulation GUI.
        fieldSim = new Field2d();
        SmartDashboard.putData("Field", fieldSim);

        RobotController.setBrownoutVoltage(7.0);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        fieldSim.setRobotPose(drivetrainSimulator.getPose());
    }

    @Override
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the simulation,
        // and write the simulated positions and velocities to our simulated encoder and
        // gyro.
        // We negate the right side so that positive voltages make the right side
        // move forward.
        // print lateralMotor voltage
        drivetrainSimulator.setInputs(
            leftMotors.get() * RobotController.getBatteryVoltage(),
            rightMotors.get() * RobotController.getBatteryVoltage(),
            lateralMotors.get() * RobotController.getBatteryVoltage());
        drivetrainSimulator.update(0.020);

        leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
        leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
        rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());
        lateralEncoderSim.setDistance(drivetrainSimulator.getLateralPositionMeters());
        lateralEncoderSim.setRate(drivetrainSimulator.getLateralVelocityMetersPerSecond());
        gyroSim.setAngle(-drivetrainSimulator.getHeading().getDegrees());

        poseEstimator.update(
            Rotation2d.fromDegrees(-gyro.getAngle()),
            getLeftEncoder().getDistance(),
            getRightEncoder().getDistance(),
            getLateralEncoder().getDistance()
        );

        SmartDashboard.putNumber("Debug/Vision Uncertainty", logScaler(getPose().getX()));
    }

    /**
     * Returns the current being drawn by the drivetrain. This works in SIMULATION
     * ONLY! If you want
     * it to work elsewhere, use the code in
     * {@link HDrivetrainSim#getCurrentDrawAmps()}
     *
     * @return The drawn current in Amps.
     */
    public double getDrawnCurrentAmps() {
        return drivetrainSimulator.getCurrentDrawAmps();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public HDriveWheelSpeeds getWheelSpeeds() {
        return new HDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate(), lateralEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        drivetrainSimulator.setPose(pose);
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
    public void tankDrive(double right, double left, double lateral) {
        drive.tankDrive(right, left, lateral);
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
     * @param leftVolts    the commanded left output
     * @param rightVolts   the commanded right output
     * @param lateralVolts the commanded lateral output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts, double lateralVolts) {
        // print out left and right volts
        tankDrive(leftVolts / 12, rightVolts / 12, lateralVolts / 12);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        lateralEncoder.reset();
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    /**
     * Gets the lateral drive encoder.
     *
     * @return the lateral drive encoder
     */
    public Encoder getLateralEncoder() {
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
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public Pose2d getPose() {
        return drivetrainSimulator.getPoseMeters();
    }

    public void displayTrajectory(Trajectory traj) {
        fieldSim.getObject("traj").setTrajectory(traj);
    }

    public static Pose2d getTruePose() {
        return drivetrainSimulator.getPoseMeters();
    }
}