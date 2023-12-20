package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.packages.InverseKinematicsRER.InverseKinematicsCalc;
import frc.packages.pathfinding.Structures.Vector;
import frc.packages.pathfinding.Structures.Vertex;
import frc.robot.Constants;
import frc.robot.supers.ArmSuper;
import frc.robot.supers.HDriveSuper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ArmCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ArmSuper m_subsystem;
    public Pose3d targetCube;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArmCommand(ArmSuper subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    public void setTarget(Pose3d target) {
        targetCube = target;
    }

    public void setIdle() {
        targetCube = null;
        idle();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (targetCube == null) {
            idle();
            return;
        }
        // Robot Position
        Pose2d globalRobot = HDriveSuper.poseEstimator.getEstimatedPosition();

        Vertex targetCube2D = new Vertex(targetCube.getX(), targetCube.getY());

        // Vector from the robot to the cube
        Vector RelCube = targetCube2D.createVector(new Vertex(globalRobot));
        // Vector facing in the forward direction of the robot
        Vector RelRobot = new Vector(Math.cos(globalRobot.getRotation().getRadians()), Math.sin(globalRobot.getRotation().getRadians())).normalize();
        // Dot product will calculate the x distance from the robot to the cube, we can assume the rotation controller will point us at the cube
        Vertex cube = new Vertex(-RelCube.dotProduct(RelRobot), targetCube.getZ());
        // This allows the robot to reach forwards or backwards
        double angleOfApproach = Math.toRadians(((10 - 90) * -Math.signum(cube.x)) + 90);
        // Update the arm
        m_subsystem.setArmMechanism(InverseKinematicsCalc.calculateTargetAngles(new Pose2d(0, 0, new Rotation2d(0)), cube, angleOfApproach));
        // Display the arm
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private void idle() {
        m_subsystem.setArmMechanism(Constants.ArmPositions.idleFront);
    }
}