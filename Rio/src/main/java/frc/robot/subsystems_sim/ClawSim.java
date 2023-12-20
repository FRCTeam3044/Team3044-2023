package frc.robot.subsystems_sim;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.supers.ClawSuper;

public class ClawSim extends ClawSuper {
    public boolean isOpen;

    public void setRollers(double speed) {
        SmartDashboard.putNumber("Claw/Roller P. Out", speed);
    }

    public void toggleClaw(boolean open) {
        isOpen = open;
        SmartDashboard.putBoolean("Claw/Open", isOpen);
    }

    public void toggleClaw() {
        isOpen = !isOpen;
        toggleClaw(isOpen);
    }
}