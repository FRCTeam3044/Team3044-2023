package frc.robot.subsystems_sim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.packages.HDrive.HDriveKinematics;
import frc.packages.HDrive.HDriveWheelSpeeds;
import frc.packages.pathfinding.Structures.Vector;
import frc.packages.util.TriConsumer;
import frc.robot.Constants.DriveConstants;
import frc.robot.supers.LowLevelVelocitySuper;

import java.util.function.Supplier;

public class LowLevelVelocitySim extends LowLevelVelocitySuper {
    private final Timer m_timer = new Timer();
    private final SimpleMotorFeedforward m_feedforward;
    private final HDriveKinematics m_kinematics;
    private final Supplier<HDriveWheelSpeeds> m_speeds;
    private final PIDController m_leftController;
    private final PIDController m_rightController;
    private final PIDController m_lateralController;
    private final TriConsumer<Double, Double, Double> m_output;
    private final Supplier<Pose2d> m_pose;
    private HDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    private Vector dir = new Vector(0, 0);
    private Vector targetDir = new Vector(0, 0);

    public final double initThreshold = 0.1;

    public LowLevelVelocitySim(
        Supplier<Pose2d> getPose,
        SimpleMotorFeedforward feedforward,
        HDriveKinematics kinematics,
        Supplier<HDriveWheelSpeeds> wheelSpeeds,
        PIDController leftController,
        PIDController rightController,
        PIDController lateralController,
        TriConsumer<Double, Double, Double> outputVolts) {
        m_pose = getPose;
        m_feedforward = feedforward;
        m_kinematics = kinematics;
        m_speeds = wheelSpeeds;
        m_leftController = leftController;
        m_rightController = rightController;
        m_lateralController = lateralController;
        m_output = outputVolts;
        m_prevTime = -1;
        m_leftController.reset();
        m_rightController.reset();
        m_lateralController.reset();
    }

    public void initialize() {
        m_prevSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(dir.x, dir.y, omegaRadiansPerSecond));
        m_timer.reset();
        m_timer.start();
        m_prevTime = m_timer.get();
    }

    public void setTargetDir(Vector targetDir) {
        double theta = -m_pose.get().getRotation().getRadians();
        this.targetDir = targetDir.rotate(theta).ensureNotNaN("Warning: Invalid direction passed to velocity controller").scale(speed);
    }

    public void setEnabled(boolean enabled) {
        if (enabled && !m_enabled) {
            initialize();
        }
        m_enabled = enabled;
    }

    public void execute() {
        if (!m_enabled) return;
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;
        //   if(dt > initThreshold){
        //     initialize();
        //     return;
        // }

        if (m_prevTime < 0) {
            m_output.accept(0.0, 0.0, 0.0);
            m_prevTime = curTime;
            return;
        }
        dir = accelLimit(targetDir, dir, DriveConstants.kMaxAcceleration).ensureNotNaN("Warning: Invalid direction passed to velocity controller");
        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(dir.x, dir.y, omegaRadiansPerSecond));

        if (dir.zero() && omegaRadiansPerSecond == 0) {
            m_output.accept(0.0, 0.0, 0.0);
            return;
        }

        SmartDashboard.putNumber("Low Level Controllers/left", targetWheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Low Level Controllers/right", targetWheelSpeeds.rightMetersPerSecond);
        SmartDashboard.putNumber("Low Level Controllers/lateral", targetWheelSpeeds.lateralMetersPerSecond);

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;
        var lateralSpeedSetpoint = targetWheelSpeeds.lateralMetersPerSecond;

        double leftOutput;
        double rightOutput;
        double lateralOutput;

        double leftFeedforward =
            m_feedforward.calculate(
                leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

        double rightFeedforward =
            m_feedforward.calculate(
                rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

        double lateralFeedforward = m_feedforward.calculate(
            lateralSpeedSetpoint, (lateralSpeedSetpoint - m_prevSpeeds.lateralMetersPerSecond) / dt);

        double latspd = m_speeds.get().lateralMetersPerSecond;
        double rightspd = m_speeds.get().rightMetersPerSecond;
        double leftspd = m_speeds.get().leftMetersPerSecond;
        leftOutput = leftFeedforward + m_leftController.calculate(leftspd, leftSpeedSetpoint);
        rightOutput = rightFeedforward + m_rightController.calculate(rightspd, rightSpeedSetpoint);
        lateralOutput = lateralFeedforward + m_lateralController.calculate(latspd, lateralSpeedSetpoint);

        // SmartDashboard.putNumber("Field/lateralOutput", latspd - lateralSpeedSetpoint);
        // SmartDashboard.putNumber("Field/leftOutput", rightspd - leftSpeedSetpoint);
        // SmartDashboard.putNumber("Field/rightOutput", leftspd - rightSpeedSetpoint);

        if (Double.isNaN(rightOutput) || Double.isNaN(leftOutput) || Double.isNaN(lateralOutput)) {
            System.out.println("Warning: Velocity calculations resulted in NaN");
            rightOutput = leftOutput = lateralOutput = 0;
        }
        rightOutput = Math.max(-12, Math.min(12, rightOutput));
        leftOutput = Math.max(-12, Math.min(12, leftOutput));
        lateralOutput = Math.max(-12, Math.min(12, lateralOutput));
        m_output.accept(leftOutput, rightOutput, lateralOutput);
        m_prevSpeeds = targetWheelSpeeds;
        m_prevTime = curTime;
    }

    public void end() {
        targetDir = new Vector(0, 0);
    }
}