package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.packages.pathfinding.Structures.Vector;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.supers.HDriveSuper;
import frc.robot.supers.LowLevelVelocitySuper;

public class FieldRelative extends CommandBase {
    // The usual driving speed
    public double SPEED = 11;
    public double SLOW_SPEED = 2.5;
    private final XboxController m_driverController;
    private final LowLevelVelocitySuper m_lowLevelVelocity;
    private final LowLevelRotation m_lowLevelRotation;
    public boolean manualRot = true;
    public boolean useAutoRot = false;
    public boolean slowDrive = false;
    private double prevRot = 0;

    public FieldRelative(LowLevelVelocitySuper lowLevelVelocity, LowLevelRotation lowLevelRotation, XboxController driverController, HDriveSuper drive) {
        m_driverController = driverController;
        m_lowLevelVelocity = lowLevelVelocity;
        m_lowLevelRotation = lowLevelRotation;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        //useAutoRot = true;
        if (RobotContainer.singleton.m_armCommand != null) RobotContainer.singleton.m_armCommand.setIdle();
        RobotContainer.singleton.m_lowLevelVelocity.setEnabled(true);
        RobotContainer.singleton.m_lowLevelVelocity.speed = SPEED;
        prevRot = HDriveSuper.poseEstimator.getEstimatedPosition().getRotation().getRadians();
    }

    @Override
    public void execute() {
        if(slowDrive){
            RobotContainer.singleton.m_lowLevelVelocity.speed = SLOW_SPEED;
        } else {
            double trigger = m_driverController.getLeftTriggerAxis();
            if(trigger < 0.1){
                trigger = 0;
            }
            RobotContainer.singleton.m_lowLevelVelocity.speed = SPEED - ((SPEED - SLOW_SPEED) * trigger);
        }
        
        if (m_driverController.getRightBumperPressed() && manualRot) {
            manualRot = false;
        }
        double fwd;
        double lat;
        double xRot;
        double yRot;
        if (DriverStation.getAlliance() == Alliance.Red) {
            xRot = m_driverController.getRightX();
            yRot = -m_driverController.getRightY();
            fwd = m_driverController.getLeftX();
            lat = -m_driverController.getLeftY();
        } else {
            xRot = -m_driverController.getRightX();
            yRot = m_driverController.getRightY();
            fwd = -m_driverController.getLeftX();
            lat = m_driverController.getLeftY();
        }
        fwd = fwd * fwd * Math.signum(fwd);
        lat = lat * lat * Math.signum(lat);

        if (!manualRot && (Math.abs(xRot) > Constants.OIConstants.kRotationDeadband || Math.abs(yRot) > Constants.OIConstants.kRotationDeadband)) {
            manualRot = true;
        }

        if (Math.abs(fwd) < Constants.OIConstants.kDriverDeadband) {
            fwd = 0;
        }
        if (Math.abs(lat) < Constants.OIConstants.kDriverDeadband) {
            lat = 0;
        }
        double rot;
        if (manualRot) {
            rot = MathUtil.angleModulus(Math.atan2(yRot, xRot) + (0.5 * Math.PI));
            SmartDashboard.putNumber("TargetRotation", rot);
            if (Math.abs(xRot) < Constants.OIConstants.kRotationDeadband && Math.abs(yRot) < Constants.OIConstants.kRotationDeadband) {
                rot = prevRot;
            }
        } else {
            double fwdRot = fwd;
            double latRot = lat;
            if (Math.abs(fwd) < Constants.OIConstants.kDriverRotationDeadband) {
                fwdRot = 0;
            }
            if (Math.abs(lat) < Constants.OIConstants.kDriverRotationDeadband) {
                latRot = 0;
            }
            if (fwdRot == 0 && latRot == 0) {
                rot = prevRot;
            } else {
                rot = Math.atan2(fwdRot, latRot);
            }
        }
        prevRot = rot;

        m_lowLevelVelocity.setTargetDir(new Vector(lat, fwd));
        //m_lowLevelVelocity.omegaRadiansPerSecond = m_driverController.getRightX() * 4;

        if (manualRot || useAutoRot) m_lowLevelRotation.setTargetRotation(new Rotation2d(rot));
        SmartDashboard.putNumber("RadiansPerSecond", m_lowLevelVelocity.omegaRadiansPerSecond);

    }

    @Override
    public void end(boolean interrupted) {
        manualRot = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}