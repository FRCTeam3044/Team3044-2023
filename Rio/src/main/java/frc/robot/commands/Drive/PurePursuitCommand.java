package frc.robot.commands.Drive;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.packages.pathfinding.Structures.Vector;
import frc.packages.purePursuit.PurePursuit;
import frc.robot.supers.LowLevelVelocitySuper;

import java.util.function.Supplier;

public class PurePursuitCommand extends CommandBase {
    public static final double MIN_ROT_ALIGN_DIST = 1;
    public double INTERPOLATION_DIST = 1.2;
    private final Timer m_timer = new Timer();
    private final Supplier<Pose2d> m_pose;
    private final PurePursuit m_follower;
    private final LowLevelVelocitySuper m_lowLevelVelocity;
    private final LowLevelRotation m_lowLevelRotation;
    private final PIDController m_pathAccuracyController;
    private final double m_speed;

    private double totalPathDist;

    private boolean backwards = false;
    private boolean alignRot = true;

    public PurePursuitCommand(
        Supplier<Pose2d> pose,
        PurePursuit controller,
        LowLevelVelocitySuper lowLevelVelocity,
        LowLevelRotation lowLevelRotation,
        PIDController pathAccuracyController,
        double speed,
        Subsystem... requirements) {
        m_pose = requireNonNullParam(pose, "pose", "PurePursuitCommand");
        m_follower = requireNonNullParam(controller, "controller", "PurePursuitCommand");
        m_lowLevelVelocity = requireNonNullParam(lowLevelVelocity, "lowLevelVelocity", "PurePursuitCommand");
        m_pathAccuracyController = requireNonNullParam(pathAccuracyController, "pathAccuracyController", "PurePursuitCommand");
        m_speed = requireNonNullParam(speed, "speed", "PurePursuitCommand");
        m_lowLevelRotation = requireNonNullParam(lowLevelRotation, "lowLevelRotation", "PurePursuitCommand");
        addRequirements(requirements);
    }

    public void setBackwards(boolean backwards) {
        //this.backwards = backwards;
    }

    @Override
    public void initialize() {
        Vector dir = m_follower.update(m_pose.get(), m_pathAccuracyController);
        m_lowLevelVelocity.setTargetDir(dir);
        m_lowLevelVelocity.speed = m_speed;
        totalPathDist = m_follower.totalPathDist();
        alignRot = totalPathDist > MIN_ROT_ALIGN_DIST;
        if (alignRot)
            m_lowLevelRotation.setTargetRotation(new Rotation2d(MathUtil.angleModulus(Math.atan2(dir.y, dir.x) + (backwards ? Math.PI : 0))));
        if (INTERPOLATION_DIST * 2 > totalPathDist) INTERPOLATION_DIST = totalPathDist / 2;
    }

    @Override
    public void execute() {
        Vector dir = m_follower.update(m_pose.get(), m_pathAccuracyController);
        double dist = m_follower.distRemaining(m_pose.get());
        if (dist < INTERPOLATION_DIST) {
            m_lowLevelVelocity.speed = m_speed * Math.min((dist + 0.45) / INTERPOLATION_DIST, 1);
        } else if (totalPathDist - dist < INTERPOLATION_DIST) {
            m_lowLevelVelocity.speed = m_speed * Math.min((totalPathDist - dist + 0.45) / INTERPOLATION_DIST, 1);

        } else m_lowLevelVelocity.speed = m_speed;

        m_lowLevelVelocity.setTargetDir(dir);
        if (alignRot) m_lowLevelRotation.setTargetRotation(new Rotation2d(
            MathUtil.angleModulus(Math.atan2(dir.y, dir.x) + (backwards ? Math.PI : 0))));
    }

    @Override
    public void end(boolean interrupted) {
        backwards = false;
        m_timer.stop();

        if (interrupted) {
            m_lowLevelVelocity.end();
        }
    }

    @Override
    public boolean isFinished() {
        if (m_follower.finished) {
            m_lowLevelVelocity.end();
            return true;
        }
        return false;
    }
}
