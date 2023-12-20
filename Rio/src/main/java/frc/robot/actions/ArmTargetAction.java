package frc.robot.actions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.packages.pathfinding.Structures.Vector;
import frc.packages.pathfinding.Structures.Vertex;
import frc.packages.util.MathUtils;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.Drive.FieldRelative;
import frc.robot.commands.Drive.LowLevelRotation;
import frc.robot.supers.HDriveSuper;

public class ArmTargetAction implements Action {
    public final double MAX_TARGET_ANGLE_DIFF = 10;
    public final double MIN_DIST = 0.5;

    private final ArmCommand m_armCommand;
    private final LowLevelRotation m_lowLevelRot;
    private final FieldRelative m_fieldRelative;
    private final Pose3d m_target;
    private final Vertex robotTarget;

    private final Vertex target2D;

    private boolean started = false;
    private boolean done = false;
    private boolean isManual = false;

    public ArmTargetAction(ArmCommand armCommand, LowLevelRotation lowLevelRot, FieldRelative fieldRelative, Pose3d target, Vertex robotTarget) {
        m_armCommand = armCommand;
        m_target = target;
        m_lowLevelRot = lowLevelRot;
        m_fieldRelative = fieldRelative;
        this.robotTarget = robotTarget;
        target2D = new Vertex(target.getX(), target.getY());
    }

    @Override
    public void start() {
        started = true;
    }

    @Override
    public void end() {
        m_armCommand.setIdle();
    }

    @Override
    public void execute() {
        Pose2d globalRobot = HDriveSuper.poseEstimator.getEstimatedPosition();
        Vector RelCube = target2D.createVector(new Vertex(globalRobot));
        boolean isInRange = robotTarget.distance(new Vertex(globalRobot)) < MIN_DIST;
        if (isManual && isInRange) {
            m_fieldRelative.useAutoRot = false;
            m_lowLevelRot.setTargetRotation(new Rotation2d(Math.atan2(RelCube.y, RelCube.x)));
        }
        if (MathUtils.angleDistance(Math.atan2(RelCube.y, RelCube.x), globalRobot.getRotation().getRadians()) < Math.toRadians(MAX_TARGET_ANGLE_DIFF) && isInRange) {
            done = true;
            m_armCommand.setTarget(m_target);
        } else {
            m_armCommand.setIdle();
        }
    }

    @Override
    public boolean isDone() {
        // TODO: Here is where we would check if the claw has scored
        return done;
    }

    @Override
    public boolean isStarted() {
        return started;
    }

    public CANCEL_CONDITION getCancelCondition() {
        return CANCEL_CONDITION.ROTATE;
    }

    @Override
    public void alertManual() {
        isManual = true;
    }
}
