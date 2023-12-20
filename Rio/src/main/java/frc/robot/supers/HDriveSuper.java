package frc.robot.supers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.packages.HDrive.HDrivePoseEstimator;
import frc.packages.HDrive.HDriveWheelSpeeds;
import frc.packages.HDrive.HDrivetrainDrive;

import com.revrobotics.CANSparkMax;

public abstract class HDriveSuper extends SubsystemBase {
    public abstract void resetEncoders();

    public abstract HDriveWheelSpeeds getWheelSpeeds();

    public abstract void resetOdometry(Pose2d pose);

    public abstract void arcadeDrive(double fwd, double rot, double lateral);

    public abstract void tankDrive(double right, double left, double lateral);

    public abstract void driveVolts(double leftVolts, double rightVolts, double lateralVolts);

    public abstract void tankDriveVolts(double leftVolts, double rightVolts, double lateralVolts);

    public abstract Pose2d getPose();

    public abstract void toggleLateralDrive();

    public abstract void engageLateralDrive(boolean engage);

    public double logScaler(double PoseX) {
        double minimumUncertainty = 0.01;
        double maximumUncertainty = 1.5;
        double a = -5.2;
        double b = 1.4;
        double fieldCenterX = 8.25;
        return ((maximumUncertainty - minimumUncertainty) / (1 + Math.pow(Math.E, (a + b * Math.abs(PoseX - fieldCenterX))))) + minimumUncertainty;
    }

    public static HDrivePoseEstimator poseEstimator;
    public CANSparkMax leftMotor1;
    public CANSparkMax leftMotor2;
    public CANSparkMax rightMotor1;
    public CANSparkMax rightMotor2;
    public CANSparkMax lateralMotor1;
    public CANSparkMax lateralMotor2;
    public static DoubleSolenoid lateralSolenoid1;
    public static DoubleSolenoid lateralSolenoid2;
    public MotorControllerGroup leftMotors;
    public MotorControllerGroup rightMotors;
    public MotorControllerGroup lateralMotors;
    public HDrivetrainDrive drive;
    public ADXRS450_Gyro gyro;
    public boolean up;
}