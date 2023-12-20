package frc.robot.supers;

import frc.packages.pathfinding.Structures.Vector;
import frc.packages.pathfinding.Structures.Vertex;

public abstract class LowLevelVelocitySuper {
    public abstract void setEnabled(boolean enabled);

    public abstract void setTargetDir(Vector dir);

    public abstract void execute();

    public abstract void end();

    public boolean m_enabled = true;
    public double speed = 5;
    public double omegaRadiansPerSecond = 0;

    public Vector accelLimit(Vector nextDir, Vector curDir, double maxAccel) {
        Vertex p1 = new Vertex(curDir.x, curDir.y);
        Vertex p2 = new Vertex(nextDir.x, nextDir.y);
        Vector accel = p2.createVector(p1);
        if (accel.magnitude() > maxAccel) {
            accel = accel.normalize().scale(maxAccel);
            p1 = p1.moveByVector(accel);
            return new Vector(p1.x, p1.y);
        }
        return nextDir;
    }
}
