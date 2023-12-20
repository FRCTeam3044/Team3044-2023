package frc.packages.purePursuit;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.packages.pathfinding.Structures.Path;
import frc.packages.pathfinding.Structures.Vector;
import frc.packages.pathfinding.Structures.Vertex;

public class PurePursuit {
    private final double lookAhead = 0.5;
    public static final double maxDistToEnd = 0.14;

    public Path path;
    private double[] distances;
    private int prevGoal = 0;

    public boolean finished = false;

    public PurePursuit(Path path) {
        this.path = path;
        distances = new double[Math.max(path.size() - 1, 0)];
        for (int i = 0; i < path.size() - 1; i++) {
            distances[i] = path.get(i).distance(path.get(i + 1));
        }
    }

    public Vector update(Pose2d currentLocationPose, PIDController controller) {
        ArrayList<Vertex> fullPath = new ArrayList<>();
        fullPath.add(path.start);
        fullPath.addAll(path);
        fullPath.add(path.target);

        Vertex robotPos = new Vertex(currentLocationPose);
        SmartDashboard.putNumberArray("Pathfinding current robot", new double[]{robotPos.x, robotPos.y, robotPos.rotation.getDegrees()});
        Vertex nextPoint = path.target;
        double dist = 1;
        boolean hasFoundPoint = false;
        Vertex closest = path.target;
        double closestDistance = robotPos.distance(path.target);

        // The goal here is to find the point furthest along the path still within the lookahead distance.
        // Circular intersections have been skipped here because the path is subdivided enough to make them unneccesary.
        for (int i = prevGoal; i < fullPath.size(); i++) {
            Vertex vertex = fullPath.get(i);
            double curDist = robotPos.distance(vertex);
            if (curDist < lookAhead) {
                dist = curDist;
                nextPoint = vertex;
                prevGoal = Math.max(i - 2, 0);
                hasFoundPoint = true;
            }
            if (curDist < closestDistance) {
                closest = vertex;
                closestDistance = curDist;
                // If the robot is too far off the path, set the closest point as the goal point
                if (!hasFoundPoint) nextPoint = closest;
            }
        }

        // TODO: fix this
        if (nextPoint == fullPath.get(fullPath.size() - 1)) {
            if (dist < maxDistToEnd) {
                finished = true;
                return new Vector(0, 0);
            } else {
                return nextPoint.createVector(robotPos).normalize().ensureNotNaN("Warning: Pure Pursuit resulted in NaN");
            }

        }

        // Kick the robot back onto the path faster if it gets too far off.
        double output = controller.calculate(closestDistance);
        //SmartDashboard.putNumber("Field/adjustment", closestDistance);
        Vector adjustmentDir = closest.createVector(robotPos).normalize().scale(output);

        Vector originalVector = nextPoint.createVector(robotPos);

        // Subtract to account for the fact that distance is alwasy positive, so PID output is always negative
        Vector adjustedDir = originalVector.subtract(adjustmentDir).normalize().ensureNotNaN("Warning: Pure Pursuit resulted in NaN");
        if (adjustedDir.zero()) adjustedDir = originalVector;

        // Debug code
        Vertex target = robotPos.moveByVector(adjustedDir);
        SmartDashboard.putNumberArray("Field/PurePursuit", new double[]{robotPos.x, robotPos.y, target.x, target.y});

        adjustedDir.x = adjustedDir.y;
        adjustedDir.y = adjustedDir.x;
        return adjustedDir.ensureNotNaN("Warning: Pure Pursuit resulted in NaN");
    }


    public double distRemaining(Pose2d robot) {
        if (path == null) return 0;
        Vertex robotPos = new Vertex(robot);
        double closestDist = 100;
        int closestPoint = 0;

        for (int i = 0; i < path.size(); i++) {
            double dist = robotPos.distance(path.get(i));
            if (dist < closestDist) {
                closestDist = dist;
                closestPoint = i;
            }
        }
        return distFromIndex(closestPoint);
    }

    public double totalPathDist() {
        double dist = 0;
        for (double d : distances) {
            dist += d;
        }
        return dist;
    }

    private double distFromIndex(int index) {
        double dist = 0;
        for (int i = index; i < distances.length; i++) {
            dist += distances[i];
        }
        return dist;
    }
}
