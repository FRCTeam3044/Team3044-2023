package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.packages.HDrive.HDriveKinematics;
import frc.packages.pathfinding.Structures.Vector;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GearboxConstants;
import frc.robot.supers.LowLevelVelocitySuper;

import java.util.function.Supplier;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

public class LowLevelVelocity extends LowLevelVelocitySuper {
    private final HDriveKinematics m_kinematics;
    private final SparkMaxPIDController m_leftController;
    private final SparkMaxPIDController m_rightController;
    private final SparkMaxPIDController m_lateralController;
    private final Supplier<Pose2d> m_pose;

    private Vector dir = new Vector(0, 0);
    private Vector targetDir = new Vector(0, 0);

    public final double initThreshold = 0.1;

    private double targetLeftVel = 0;
    private double targetRightVel = 0;
    private double targetLatVel = 0;

    private double prevLatVel = 0;

    public LowLevelVelocity(
        Supplier<Pose2d> getPose,
        HDriveKinematics kinematics,
        SparkMaxPIDController leftController,
        SparkMaxPIDController rightController,
        SparkMaxPIDController lateralController) {
        m_pose = getPose;
        m_kinematics = kinematics;
        m_leftController = leftController;
        m_rightController = rightController;
        m_lateralController = lateralController;
        m_leftController.setReference(0, CANSparkMax.ControlType.kVelocity);
        m_rightController.setReference(0, CANSparkMax.ControlType.kVelocity);
        m_lateralController.setReference(0, CANSparkMax.ControlType.kVelocity);
    }

    public void setTargetDir(Vector targetDir) {
        double theta = m_pose.get().getRotation().getRadians();
        this.targetDir = targetDir.rotate(theta).ensureNotNaN("Warning: Invalid direction passed to velocity controller").scale(speed);
    }

    public void setEnabled(boolean enabled) {
        m_enabled = enabled;
    }

    public void execute() {
        if (!m_enabled) return;

        SmartDashboard.putNumber("leftMotorSetpoint", targetLeftVel);
        SmartDashboard.putNumber("rightMotorSetpoint", targetRightVel);
        SmartDashboard.putNumber("latMotorSetpoint", targetLatVel);
        SmartDashboard.putNumber("leftMotorOutput", RobotContainer.singleton.m_robotDrive.leftMotor1.getEncoder().getVelocity());
        SmartDashboard.putNumber("rightMotorOutput", RobotContainer.singleton.m_robotDrive.rightMotor1.getEncoder().getVelocity());
        SmartDashboard.putNumber("latMotorOutput", RobotContainer.singleton.m_robotDrive.lateralMotor1.getEncoder().getVelocity());
        SmartDashboard.putNumber("leftMotorVolts", RobotContainer.singleton.m_robotDrive.leftMotor1.get());

        dir = accelLimit(targetDir, dir, DriveConstants.kMaxAcceleration).ensureNotNaN("Warning: Invalid direction passed to velocity controller");
        if (dir.zero() && omegaRadiansPerSecond == 0) {
            m_leftController.setReference(0, CANSparkMax.ControlType.kVelocity);
            m_rightController.setReference(0, CANSparkMax.ControlType.kVelocity);
            m_lateralController.setReference(0, CANSparkMax.ControlType.kVelocity);
            return;
        }

        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(dir.x, dir.y, omegaRadiansPerSecond));
        SmartDashboard.putNumber("llv dir x", dir.x);
        SmartDashboard.putNumber("llv dir y", dir.y);
        SmartDashboard.putNumber("Low Level Controllers/left", targetWheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Low Level Controllers/right", targetWheelSpeeds.rightMetersPerSecond);
        SmartDashboard.putNumber("Low Level Controllers/lateral", targetWheelSpeeds.lateralMetersPerSecond);


        setLeftVelocity(targetWheelSpeeds.leftMetersPerSecond);
        setRightVelocity(targetWheelSpeeds.rightMetersPerSecond);
        setLateralVelocity(targetWheelSpeeds.lateralMetersPerSecond);
    }

    public void end() {
        targetDir = new Vector(0, 0);
    }


    public void setLeftVelocity(double target) {
        double leftRPM = (60 / (2 * Math.PI * DriveConstants.leftWheelRadius)) * GearboxConstants.kDriveRatio * target;
        leftRPM = NaNCheck(leftRPM);
        m_leftController.setReference(leftRPM, CANSparkMax.ControlType.kVelocity);
        targetLeftVel = leftRPM;
    }

    public void setRightVelocity(double target) {
        double rightRPM = (60 / (2 * Math.PI * DriveConstants.rightWheelRadius)) * GearboxConstants.kDriveRatio * target;
        rightRPM = NaNCheck(rightRPM);
        m_rightController.setReference(rightRPM, CANSparkMax.ControlType.kVelocity);
        targetRightVel = rightRPM;
    }

    public void setLateralVelocity(double target) {
        // TODO: Lat acceleration
        // if(Math.abs(target - prevLatVel) > DriveConstants.kMaxLateralAcceleration){
        //     target = pr
        // }
        prevLatVel = target;
        double lateralRPM = (60 / (2 * Math.PI * DriveConstants.lateralWheelRadius)) * GearboxConstants.kLateralRatio * target;
        lateralRPM = NaNCheck(lateralRPM);
        m_lateralController.setReference(lateralRPM, CANSparkMax.ControlType.kVelocity);
        targetLatVel = lateralRPM;
    }

    private double NaNCheck(double val) {
        if (Double.isNaN(val)) {
            System.out.println("Warning: Velocity calculations resulted in NaN");
            return 0;
        }
        return val;
    }
}
