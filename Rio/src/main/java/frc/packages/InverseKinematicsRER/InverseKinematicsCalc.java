package frc.packages.InverseKinematicsRER;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.packages.pathfinding.Structures.Vertex;
import frc.robot.Constants.ArmConstants;

public class InverseKinematicsCalc {
    public static ArmPosition calculateTargetAngles(Pose2d robotPose, Vertex cubeLocation, double angleOfApproach){
        /*  x = the x plane
            y = the y plane
            d = distance from the center of the gamepiece to the center of the bot along the ground
            h = height of the shoulder
            a = Distance from the shoulder joint to the claw joint
            b = Distance from the claw joint to the center of the gamepiece
            g = Distance from the ground to the center of the robot
            z = Angle of the distance from the claw joint to the center of the gamepiece (b) and the x plane (x)
            w = Angle of the height of the shoulder (h) and the distance from the shoulder joint to the claw joint (a)

            Length of x → x=bcos(z)
            Length of y → y=bsin(z)
            Length of a → (d-x)2 + (h - g - y)2 = a2
            Angle w → -1sin((d - x) / a)
            Angle z → -1cos(x / b) 
        */


        double d = robotPose.getX() - cubeLocation.x;
        double b = ArmConstants.wristToGamePiece;
        double x = b * Math.cos(angleOfApproach);
        double y = b * Math.sin(angleOfApproach);
        double a = Math.sqrt((d-x)*(d-x) + (ArmConstants.shoulderHeight - cubeLocation.y - y)*(ArmConstants.shoulderHeight - cubeLocation.y - y));
        double w = Math.asin((d - x) / a);

        double wrist = Math.toDegrees(MathUtil.angleModulus((0.5 * Math.PI) - angleOfApproach - w));
        


        // Constrain the translational joint
        a = Math.min(Math.max(a, ArmConstants.minTranslation), ArmConstants.maxTranslation);


        w = Math.toDegrees(MathUtil.angleModulus(w - (0.5 * Math.PI)));

        // If the target is above the shoulder, flip the rotations and ensure the shoulder doesn't go to high
        boolean isAboveShoulder = ArmConstants.shoulderHeight < cubeLocation.y;
        w = isAboveShoulder ? Math.min(-w, ArmConstants.maxShoulderAngle) : w;
        

        wrist = isAboveShoulder ? -wrist : wrist;
        return new ArmPosition(w, a, wrist);
    }
}
