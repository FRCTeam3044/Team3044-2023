package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.packages.pathfinding.PathfindingWS;
import frc.packages.pathfinding.Structures.Node;
import frc.packages.pathfinding.Structures.Path;
import frc.packages.pathfinding.Structures.Vector;
import frc.packages.pathfinding.Structures.Vertex;
import frc.packages.util.MathUtils;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.StartPathfindCommand;
import frc.robot.commands.Drive.StartPathfindCommand.PathfindSnapMode;
import frc.robot.subsystems.HDrive;
import frc.robot.supers.HDriveSuper;

public class PickupCubeCommand extends CommandBase {
    private final StartPathfindCommand m_startPathfindCommand;
    private final PathfindingWS m_pathfinder;

    private boolean finished = false;
    private boolean scoring = false;
    private boolean initialized = false;

    private Pose3d cubeCenter;

    public final double IDEAL_PICKUP_RADIUS = 1;
    public final double MAX_PICKUP_RADIUS = 2.5;
    public final double MIN_PICKUP_RADIUS = 0.5;

    private Vertex cubeCenter2d;

    private boolean nextScoreDir = false;

    public PickupCubeCommand(StartPathfindCommand startPathfindCommand, PathfindingWS pathfinder) {
        m_startPathfindCommand = startPathfindCommand;
        m_pathfinder = pathfinder;
    }

    public void setCubeCenter(Pose3d cubeCenter) {
        this.cubeCenter = cubeCenter;
    }

    public void setScoring(boolean scoring) {
        this.scoring = scoring;
    }

    @Override
    public void initialize() {
        initialized = false;
        finished = false;
        Pose2d robotPose = HDrive.poseEstimator.getEstimatedPosition();
        Node start = new Node(robotPose.getX(), robotPose.getY());
        Vertex cubeCenter2D = new Vertex(cubeCenter.getX(), cubeCenter.getY());
        // For the sake of optimization this doesn't need to be run on scoring nodes.
        // In the event the robot is somehow inside the scoring nodes, it might cause problems.
        if (!scoring) {
            Vector robotToCube = cubeCenter2D.createVector(start);
            double dist = robotToCube.magnitude();
            if (dist < IDEAL_PICKUP_RADIUS && dist > MIN_PICKUP_RADIUS) {
                start.rotation = new Rotation2d(Math.atan2(robotToCube.y, robotToCube.x));
                RobotContainer.singleton.m_pointController.setTarget(start);
                RobotContainer.singleton.m_pointController.schedule();
                return;
            } else if (dist < MIN_PICKUP_RADIUS) {
                // The robot is too close to the cube to pickup.
                Vector toRobot = start.createVector(cubeCenter2D).normalize().scale(MIN_PICKUP_RADIUS);
                Vertex newStart = cubeCenter2D.moveByVector(toRobot);
                start = new Node(newStart.x, newStart.y);
            }
        }
        cubeCenter2d = cubeCenter2D;

        m_pathfinder.sendPathfindingRequest(start, new Node(cubeCenter2D.x, cubeCenter2D.y), scoring ? PathfindSnapMode.SNAP_TO_SCORING_NODES : PathfindSnapMode.SNAP, null);
    }

    @Override
    public void end(boolean interrupted) {
        scoring = false;
        cubeCenter2d = null;
        initialized = false;
    }

    @Override
    public void execute() {
        if (!initialized) {
            if (cubeCenter2d != null && m_pathfinder.hasRecievedPath) {
                Path toCenterCube = m_pathfinder.getPath();
                toCenterCube = removeIfTooClose(toCenterCube, cubeCenter2d, IDEAL_PICKUP_RADIUS);
                Vertex newTarget = toCenterCube.end();
                if (newTarget == null) {
                    // TODO: Add better handling for this
                    System.out.println("Unable to find path to cube (too close)");
                    finished = true;
                    return;
                }
                if (newTarget.distance(cubeCenter2d) > MAX_PICKUP_RADIUS) {
                    System.out.println(newTarget.distance(cubeCenter2d));
                    System.out.println("Unable to find path to cube (cube too far away)");
                    finished = true;
                    return;
                }
                toCenterCube.target = newTarget;
                toCenterCube.createFullPath();

                Vector finalRobotTocube = cubeCenter2d.createVector(toCenterCube.target);
                boolean backwards = false;
                if (!scoring) {
                    Vector dir = toCenterCube.get(Math.min(toCenterCube.size() - 1, 1)).createVector(toCenterCube.start);
                    double curAngle = HDriveSuper.poseEstimator.getEstimatedPosition().getRotation().getRadians();
                    double fwdAngle = Math.atan2(dir.y, dir.x);
                    double revAngle = MathUtil.angleModulus(fwdAngle + Math.PI);
                    backwards = (MathUtils.angleDistance(revAngle, curAngle) < MathUtils.angleDistance(fwdAngle, curAngle));
                    // TODO: Plug in bool from cone keypoints
                    nextScoreDir = scoringDirection(true, backwards);
                    m_startPathfindCommand.setBackwards(backwards);
                } else {
                    backwards = nextScoreDir;
                    m_startPathfindCommand.setBackwards(nextScoreDir);
                }
                toCenterCube.target.rotation = scoring ?
                    new Rotation2d(MathUtil.angleModulus((DriverStation.getAlliance() == Alliance.Red ? 0 : Math.PI) + (backwards ? Math.PI : 0))) :
                    new Rotation2d(MathUtil.angleModulus(Math.atan2(finalRobotTocube.y, finalRobotTocube.x) + (backwards ? Math.PI : 0)));
                m_startPathfindCommand.setPath(toCenterCube);
                m_startPathfindCommand.setSnapMode(scoring ? PathfindSnapMode.SNAP_TO_SCORING_NODES : PathfindSnapMode.SNAP);

                m_startPathfindCommand.schedule();

                initialized = true;
            } else if (m_pathfinder.error) {
                System.out.println("Pathfinder error");
                finished = true;
                initialized = true;
            }
        } else {
            if (RobotContainer.singleton.m_pointController.isFinished() && RobotContainer.singleton.m_followTrajectory.isFinished()) {
                finished = true;
            }
        }

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    private Path removeIfTooClose(Path path, Vertex point, double minDist) {
        path.removeIf(v -> v.distance(point) < minDist);
        return path;
    }

    /**
     * @return true if the robot should score from the back, false if it should score from the front
     */
    private boolean scoringDirection(boolean coneFacingAway, boolean botBackwards) {
        if (coneFacingAway) {
            return botBackwards;
        } else {
            return !botBackwards;
        }
    }
}