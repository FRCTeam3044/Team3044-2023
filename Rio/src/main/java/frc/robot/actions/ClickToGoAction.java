package frc.robot.actions;

import frc.packages.pathfinding.Structures.Node;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.StartPathfindCommand;
import frc.robot.commands.Drive.StartPathfindCommand.PathfindSnapMode;

// Used for click to go and hotkeys since their basically the same internally
public class ClickToGoAction implements Action {
    private StartPathfindCommand m_startPathfindCommand;
    private Node targetNode;
    private PathfindSnapMode snapMode;

    private boolean started = false;

    public ClickToGoAction(StartPathfindCommand startPathfindCommand, Node targetNode, PathfindSnapMode snapMode) {
        m_startPathfindCommand = startPathfindCommand;
        this.targetNode = targetNode;
        this.snapMode = snapMode;
    }

    @Override
    public void start() {
        RobotContainer.singleton.m_fieldRelative.cancel();
        RobotContainer.singleton.m_arcadeDrive.cancel();
        started = true;
        m_startPathfindCommand.setTarget(targetNode);
        m_startPathfindCommand.setSnapMode(snapMode);
        assert RobotContainer.singleton.m_armCommand != null;
        RobotContainer.singleton.m_armCommand.setIdle();
        m_startPathfindCommand.schedule();
    }

    @Override
    public boolean isDone() {
        return started && !RobotContainer.singleton.m_followTrajectory.isScheduled();
    }

    public void end() {
        RobotContainer.singleton.m_followTrajectory.cancel();
    }

    @Override
    public boolean isStarted() {
        return started;
    }

    @Override
    public CANCEL_CONDITION getCancelCondition() {
        return CANCEL_CONDITION.DRIVE;
    }
}
