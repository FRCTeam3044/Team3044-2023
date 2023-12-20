package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.packages.pathfinding.PathfindingWS;
import frc.packages.pathfinding.Structures.Node;
import frc.packages.pathfinding.Structures.Path;
import frc.packages.pathfinding.Structures.Vertex;
import frc.packages.purePursuit.PurePursuit;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.StartPathfindCommand.PathfindSnapMode;
import frc.robot.supers.HDriveSuper;
import frc.robot.supers.LowLevelVelocitySuper;

// Simple wrapper class to launch the PurePursuitCommand, then switch to the point controller
public class FollowTrajectory extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final HDriveSuper m_subsystem;
    private final PathfindingWS m_pathfinder;
    private final LowLevelVelocitySuper m_lowLevelVelocity;
    private final LowLevelRotation m_lowLevelRotation;
    private PurePursuit path;
    private Node target;
    private PurePursuitCommand command;

    private PathfindSnapMode snapMode = PathfindSnapMode.NONE;

    private boolean failed;
    private boolean hasSetPath;
    private boolean backwards;
    public final double MIN_UNSNAPPED_TARGET_DIST = 0.1;
    public final double SPEED = 1.5;

    public FollowTrajectory(HDriveSuper subsystem, PathfindingWS m_Pathfinder, LowLevelVelocitySuper lowLevelVelocity, LowLevelRotation lowLevelRotation) {
        // This command doesn't need to use the drive subsytem except to pass it on to PurePursuit, so it doesn't need to be a requirement
        m_subsystem = subsystem;
        m_pathfinder = m_Pathfinder;
        m_lowLevelVelocity = lowLevelVelocity;
        m_lowLevelRotation = lowLevelRotation;
    }

    public void setPath(Path path) {
        this.path = new PurePursuit(path);
        this.hasSetPath = true;
    }

    public void setTarget(Node target) {
        this.target = target;
    }

    public void setSnapMode(PathfindSnapMode snapMode) {
        this.snapMode = snapMode;
    }

    public void setBackwards(boolean backwards) {
        this.backwards = backwards;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        failed = false;
        if ((target == null || target.x < 0 || target.y < 0) && path == null) {
            failed = true;
            System.out.println("Invalid target");
            return;
        }
        RobotContainer.singleton.m_lowLevelVelocity.setEnabled(true);


        if (!hasSetPath) {
            m_pathfinder.sendPathfindingRequest(new Node(
                HDriveSuper.poseEstimator.getEstimatedPosition().getX(),
                HDriveSuper.poseEstimator.getEstimatedPosition().getY(),
                HDriveSuper.poseEstimator.getEstimatedPosition().getRotation()), target, snapMode, null);
        } else if (path != null) {
            command = new PurePursuitCommand(
                HDriveSuper.poseEstimator::getEstimatedPosition,
                path,
                m_lowLevelVelocity,
                m_lowLevelRotation,
                new PIDController(1, 0, 0.001),
                SPEED,
                m_subsystem);
            command.setBackwards(backwards);
            command.schedule();
        } else {
            hasSetPath = false;
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!hasSetPath) {
            if (m_pathfinder.hasRecievedPath) {
                Path p = m_pathfinder.getPath();
                System.out.println("snapMode: " + snapMode + " dist: " + new Vertex(HDriveSuper.poseEstimator.getEstimatedPosition()).distance(p.unsnappedTarget));
                if (snapMode == PathfindSnapMode.SNAP_TO_SCORING_NODES && new Vertex(HDriveSuper.poseEstimator.getEstimatedPosition()).distance(p.unsnappedTarget) < MIN_UNSNAPPED_TARGET_DIST) {
                    RobotContainer.singleton.m_pointController.setTarget(p.unsnappedTarget, p.getFinalRot());
                    RobotContainer.singleton.m_pointController.schedule();
                    failed = true;
                    return;
                }
                path = new PurePursuit(p);
                command = new PurePursuitCommand(
                    HDriveSuper.poseEstimator::getEstimatedPosition,
                    path,
                    m_lowLevelVelocity,
                    m_lowLevelRotation,
                    new PIDController(1, 0, 0.001),
                    SPEED,
                    m_subsystem);
                command.setBackwards(backwards);
                command.schedule();
                hasSetPath = true;
            } else if (m_pathfinder.error) {
                failed = true;
            }

        } else if (command != null && path != null && command.isFinished()) {
            Rotation2d targetRotation = path.path.target.rotation;

            if (targetRotation.getDegrees() == -2) targetRotation = path.path.getFinalRot();
            RobotContainer.singleton.m_pointController.setTarget((snapMode == PathfindSnapMode.SNAP_THEN_POINT || snapMode == PathfindSnapMode.SNAP_TO_SCORING_NODES) ? path.path.unsnappedTarget : path.path.target, targetRotation);
            RobotContainer.singleton.m_pointController.schedule();
            failed = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (command != null) command.cancel();
        m_subsystem.driveVolts(0, 0, 0);
        snapMode = PathfindSnapMode.NONE;
        path = null;
        hasSetPath = false;
        command = null;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return failed;
    }

    public double distRemaining() {
        return path == null ? 0 : path.distRemaining(HDriveSuper.poseEstimator.getEstimatedPosition());
    }

}
