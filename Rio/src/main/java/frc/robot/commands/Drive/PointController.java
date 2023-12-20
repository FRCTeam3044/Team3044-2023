package frc.robot.commands.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.packages.pathfinding.Structures.Vector;
import frc.packages.pathfinding.Structures.Vertex;
import frc.robot.supers.HDriveSuper;
import frc.robot.supers.LowLevelVelocitySuper;

public class PointController extends CommandBase {
    private final Supplier<Pose2d> m_pose;
    private final LowLevelVelocitySuper m_velController;
    private final LowLevelRotation m_rotController;
    private Vertex target;
    private Rotation2d targetRot;

    public double threshold = 0.035;
    public double minTime = 0.4;
    public double minSpeed = 0;
    public double maxSpeed = 1;
    public double transitionDistance = 0.7; // Meters

    private boolean inRange = false;
    private Timer timer = new Timer();
    private boolean done = false;

    public PointController(LowLevelVelocitySuper velController, LowLevelRotation rotController, PIDController errorController, Supplier<Pose2d> pose, HDriveSuper drive) {
        m_velController = velController;
        m_rotController = rotController;
        m_pose = pose;
        addRequirements(drive);
    }

    public void setTarget(Vertex targetPoint) {
        target = targetPoint;
        targetRot = null;
    }

    public void setTarget(Vertex targetPoint, Rotation2d rot) {
        target = targetPoint;
        targetRot = rot;
    }

    @Override
    public void initialize() {
        done = false;
        timer.reset();
        inRange = false;
    }

    @Override
    public void execute() {
        if (targetRot != null) {
            m_rotController.setTargetRotation(targetRot);
        }
        Vector dir = target.createVector(new Vertex(m_pose.get()));
        SmartDashboard.putNumber("Debug/Point Controller/X vector", dir.x);
        SmartDashboard.putNumber("Debug/Point Controller/Y vector", dir.y);
        dir.x = -dir.x;
        double error = dir.magnitude();
        
        SmartDashboard.putNumber("Debug/Point Controller/Error", error);
        double speed = (maxSpeed/transitionDistance) * error;
        if(Math.abs(speed) > maxSpeed){
            speed = maxSpeed;
        }
        if (Math.abs(speed) < minSpeed) {
            speed = minSpeed;
        }
        
        
        SmartDashboard.putNumber("Debug/Point Controller/Speed", speed);
        m_velController.speed = speed;
        m_velController.setTargetDir(dir.normalize());

        if (Math.abs(error) < threshold) {
            if (!inRange) {
                inRange = true;
                timer.reset();
                timer.start();
            } else {
                if (timer.get() > minTime) {
                    done = true;
                }
            }
        } else if (inRange) {
            inRange = false;
            timer.stop();
        }

    }

    @Override
    public void end(boolean interrupted) {
        m_velController.end();
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
