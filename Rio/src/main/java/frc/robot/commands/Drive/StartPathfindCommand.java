package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.packages.pathfinding.Structures.Node;
import frc.packages.pathfinding.Structures.Path;

// This command is in place to allow for hot-swapping paths
public class StartPathfindCommand extends CommandBase {
    private final FollowTrajectory m_followTrajectory;
    private final PointController m_pointController;

    private Node m_target;
    private Path m_path;
    private boolean backwards = false;
    private PathfindSnapMode m_snapMode = PathfindSnapMode.NONE;

    public StartPathfindCommand(FollowTrajectory followTrajectory, PointController pointController) {
        m_followTrajectory = followTrajectory;
        m_pointController = pointController;
    }

    public void setTarget(Node target) {
        m_target = target;
    }

    public void setPath(Path path) {
        m_path = path;
    }

    public void setSnapMode(PathfindSnapMode snapMode) {
        m_snapMode = snapMode;
    }

    public void setBackwards(boolean backwards) {
        this.backwards = backwards;
    }

    @Override
    public void initialize() {
        m_followTrajectory.cancel();
        m_pointController.cancel();
        if (m_path != null) {
            m_followTrajectory.setPath(m_path);
            m_path = null;
        } else if (m_target != null) {
            m_followTrajectory.setTarget(m_target);
            m_target = null;
        } else {
            double[] target = SmartDashboard.getNumberArray("TargetLocation", new double[]{-1, -1, -1});
            if (target[0] == -1 && target[1] == -1 && target[2] == -1) {
                return;
            }
            m_followTrajectory.setTarget(new Node(target[0], target[1], Rotation2d.fromDegrees(-2)));
        }
        m_followTrajectory.setSnapMode(m_snapMode);
        m_followTrajectory.setBackwards(backwards);
        m_followTrajectory.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    public enum PathfindSnapMode {
        NONE, SNAP, SNAP_THEN_POINT, SNAP_TO_SCORING_NODES
    }
}
