package frc.robot.actions;

import org.json.JSONArray;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.packages.pathfinding.Pathfinder;
import frc.packages.pathfinding.Structures.Vertex;
import frc.packages.util.ConfigLoader;
import frc.robot.RobotContainer;
import frc.robot.actions.Action.ActionGroup;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.PickupCubeCommand;
import frc.robot.commands.Drive.FieldRelative;
import frc.robot.commands.Drive.LowLevelRotation;

public class ScoreAction implements ActionGroup {
    private Action[] actions = new Action[2];
    private final Pose3d targetNode;
    private boolean started = false;

    public ScoreAction(PickupCubeCommand pickupCubeCommand, ArmCommand armCommand, LowLevelRotation lowLevelRot, FieldRelative fieldRelative, Pathfinder pathfinder, int scoringNode) {
        JSONArray targetNodes = DriverStation.getAlliance() == DriverStation.Alliance.Red ? ConfigLoader.redScoringNodes : ConfigLoader.blueScoringNodes;
        JSONArray target = targetNodes.getJSONArray(scoringNode);
        targetNode = new Pose3d(target.getDouble(0), target.getDouble(1), target.getDouble(2) + ConfigLoader.scoringNodeOffset, new Rotation3d(0, 0, 0));
        actions[0] = new GetToScoringNodeAction(pickupCubeCommand, targetNode);
        Vertex targetRobot = pathfinder.snap(new Vertex(targetNode.getX(), targetNode.getY()), true);
        actions[1] = new ArmTargetAction(armCommand, lowLevelRot, fieldRelative, targetNode, targetRobot);
    }

    @Override
    public void start() {
        RobotContainer.singleton.m_fieldRelative.cancel();
        RobotContainer.singleton.m_arcadeDrive.cancel();
        started = true;
        actions[0].start();
        actions[1].start();
    }

    @Override
    public boolean isDone() {
        return started && actions[0].isDone() && actions[1].isDone();
    }

    @Override
    public void execute() {
        actions[0].execute();
        actions[1].execute();
    }

    @Override
    public boolean isStarted() {
        return started;
    }

    @Override
    public CANCEL_CONDITION getCancelCondition() {
        return CANCEL_CONDITION.ROTATE;
    }

    @Override
    public Action[] getActions() {
        return actions;
    }
}
