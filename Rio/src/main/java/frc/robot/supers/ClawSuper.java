package frc.robot.supers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ClawSuper extends SubsystemBase {
    public abstract void toggleClaw(boolean open);

    public abstract void toggleClaw();

    public abstract void setRollers(double speed);
}