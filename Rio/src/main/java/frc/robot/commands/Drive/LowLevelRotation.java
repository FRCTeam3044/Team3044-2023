package frc.robot.commands.Drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.supers.LowLevelVelocitySuper;

public class LowLevelRotation {
    private final PIDController controller;
    private final Supplier<Pose2d> robotPos;
    private final LowLevelVelocitySuper velocityController;

    // Radians
    private final double deadband = Math.toRadians(1.5);
    // Radians per second
    public double minSpeed = 1;
    public double maxSpeed = 6;

    private Rotation2d targetRot = new Rotation2d(0);


    public LowLevelRotation(PIDController rotController, Supplier<Pose2d> robotPosition, LowLevelVelocitySuper velocity) {
        robotPos = robotPosition;
        controller = rotController;
        velocityController = velocity;
    }

    /**
     * Sets the target rotation to the given rotation.
     *
     * @param rot the target rotation
     */
    public void setTargetRotation(Rotation2d rot) {
        double targetRad = MathUtil.angleModulus(rot.getRadians());
        targetRot = new Rotation2d(targetRad);
    }

    public void execute() {
        if (!velocityController.m_enabled) return;
        double curRotation = MathUtil.angleModulus(robotPos.get().getRotation().getRadians());
        double error = targetRot.getRadians() - curRotation;
        SmartDashboard.putNumber("Target Rotation", targetRot.getRadians());
        while (Math.abs(error) > Math.PI) {
            error -= Math.signum(error) * Math.PI * 2;
        }
        if (Math.abs(error) < deadband) {
            error = 0;
        }

        double output = controller.calculate(error);
        output = Math.max(-12, Math.min(12, output));
        SmartDashboard.putNumber("Low Level Controllers/Rotation Error", error);
        if (Math.abs(output) > maxSpeed) {
            output = Math.signum(-error) * maxSpeed;
        }
        if (Math.abs(output) < minSpeed) {
            output = Math.signum(-error) * minSpeed;
        }
        velocityController.omegaRadiansPerSecond = output;
    }

}
