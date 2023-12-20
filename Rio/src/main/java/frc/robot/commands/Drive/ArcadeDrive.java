package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.supers.HDriveSuper;

public class ArcadeDrive extends CommandBase {
    private final HDriveSuper m_drive;
    private final XboxController m_driverController;

    public ArcadeDrive(HDriveSuper drive, XboxController driverController) {
        m_drive = drive;
        m_driverController = driverController;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        RobotContainer.singleton.m_armCommand.setIdle();
        RobotContainer.singleton.m_lowLevelVelocity.setEnabled(false);
    }

    @Override
    public void execute() {
        double fwd = -m_driverController.getLeftY();
        fwd = Math.min(0.6, Math.max(-0.6, fwd));
        double rot = -m_driverController.getRightX();
        rot = Math.min(0.6, Math.max(-0.6, rot));
        double lat;
        if (Math.abs(fwd) < Constants.OIConstants.kDriverDeadband) {
            fwd = 0;
        }
        if (Math.abs(rot) < Constants.OIConstants.kRotationDeadband - 0.3) {
            rot = 0;
        }

        if (m_driverController.getPOV() == 90) {
            lat = 0.3;
        } else if (m_driverController.getPOV() == 270) {
            lat = -0.3;
        } else {
            lat = 0;
        }
        m_drive.arcadeDrive(fwd, rot, lat);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
