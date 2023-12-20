package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.packages.pathfinding.Structures.Node;
import frc.packages.pathfinding.Structures.Vertex;
import frc.packages.util.ConfigLoader;
import frc.robot.actions.ClickToGoAction;
import frc.robot.actions.ScoreAction;
import frc.robot.commands.Drive.StartPathfindCommand;
import frc.robot.commands.Drive.StartPathfindCommand.PathfindSnapMode;

import java.util.Arrays;

import org.json.JSONArray;

public class InControlInterface {
    private double prevTargetNode = -1;
    private double[] prevLoc = new double[]{-2, -2, -2};

    public void setup() {
        SmartDashboard.putNumberArray("TargetLocation", new double[]{-2, -2, -2});
        boolean[] scoringNodes = new boolean[27];
        Arrays.fill(scoringNodes, false);
        SmartDashboard.putBooleanArray("ScoringNodes", scoringNodes);
    }

    /**
     * 
     */
    public void checkForUpdates() {
        // Check if pathfinding target changed, and if so pathfind to it.
        if(Robot.isSimulation()){
            double[] targetLoc = SmartDashboard.getNumberArray("TargetLocation", new double[]{-2, -2, -2});
            if (!Arrays.equals(targetLoc, prevLoc)) {
                String targetMode = SmartDashboard.getString("NavType", "click");
                StartPathfindCommand.PathfindSnapMode snapMode = StartPathfindCommand.PathfindSnapMode.SNAP;
                if (targetMode.equals("hotkey")) snapMode = PathfindSnapMode.SNAP_THEN_POINT;
                prevLoc = targetLoc;
                ClickToGoAction action = new ClickToGoAction(RobotContainer.singleton.m_startPathfind, new Node(targetLoc[0], targetLoc[1], Rotation2d.fromDegrees(targetLoc[2])), snapMode);
                RobotContainer.singleton.m_scheduler.add(action);
            }
    
            // Check if target node changed, and if so go score on it.
            double targetNode = SmartDashboard.getNumber("TargetNode", -1);
            if (prevTargetNode != targetNode) {
                prevTargetNode = targetNode;
                if (targetNode != -1) {
                    ScoreAction action = new ScoreAction(RobotContainer.singleton.m_pickupCubeCommand, RobotContainer.singleton.m_armCommand, RobotContainer.singleton.m_lowLevelRotation, RobotContainer.singleton.m_fieldRelative, RobotContainer.singleton.m_pathfinder, (int) targetNode);
                    RobotContainer.singleton.m_scheduler.add(action);
                }
            }
        } else {
            int targetNode = (int)SmartDashboard.getNumber("TargetNode", -1);
            if (targetNode != -1) {
                boolean isRed = DriverStation.getAlliance() == DriverStation.Alliance.Red;
                JSONArray targetNodes = isRed ? ConfigLoader.redScoringNodes : ConfigLoader.blueScoringNodes;
                JSONArray target = targetNodes.getJSONArray(targetNode);
                RobotContainer.singleton.m_pointController.setTarget(new Vertex(isRed ? 14.6 : 1.4, target.getDouble(1)), new Rotation2d(isRed ? 0 : Math.PI));
                RobotContainer.singleton.m_pointController.schedule();
                SmartDashboard.putNumber("TargetNode", -1);
            }
        }
    }
}