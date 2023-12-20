package frc.packages.pathfinding.Structures;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.supers.HDriveSuper;

public class Path extends ArrayList<Vertex> {
    public final static double minDist = 0.1;
    public final static double pointSpacing = 0.12;
    public final static double smoothSpacing = 0.08;
    public final static double cornerDist = 0.6;

    private ArrayList<Vertex> fullPath = new ArrayList<>();
    private ArrayList<PathSegment> segments = new ArrayList<>();

    public Vertex start;
    public Vertex target;

    public Vertex unsnappedTarget = null;

    public Path(Vertex start, Vertex target){
        super();
        this.start = start;
        this.target = target;
    }

    public void pursuitPrepare(){
        createFullPath();
        bezierSmoothing();
        injectPoints();
        updateFromSegments();
        sendToSmartDashboard();
    }

    public void bezierSmoothing(){
        // this does not include the start and endpoint, so in the case where the shortest path is a straight line it would be empty.
        if(this.size() < 1){
            segments.add(new PathSegment(start, target));
            return;
        }
        for(int i = 0; i < this.size(); i++){
            Vertex p1 = this.get(i);
            PathSegment curve = new PathSegment();
            Vertex prev = fullPath.get(i);
            // fullPath takes into account the start and endpoint while this does not, so we can garuntee that i + 2 will never be out of bounds.
            Vertex next = fullPath.get(i + 2);

            Vector prevVector = prev.createVector(p1).normalize().scale(cornerDist);
            Vector nextVector = next.createVector(p1).normalize().scale(cornerDist);

            Vertex p0 = p1.moveByVector(prevVector);
            Vertex p2 = p1.moveByVector(nextVector);

            
            for(double t = 0; t < cornerDist; t += smoothSpacing){
                Vertex q0 = p0.moveByVector(p1.createVector(p0).normalize().scale(t));
                Vertex q1 = p1.moveByVector(p2.createVector(p1).normalize().scale(t));
                Vertex pos = q0.moveByVector(q1.createVector(q0).normalize().scale(t));
                curve.add(pos);
            }
            if(i == 0){
                segments.add(new PathSegment(start, curve.start()));
            } else {
                segments.add(new PathSegment(segments.get(2 * i - 1).end(), curve.start()));
            }
            segments.add(curve);
        }
        segments.add(new PathSegment(segments.get(segments.size() - 1).end(), target));
    }

    public void updateFromSegments(){
        this.clear();
        for(PathSegment seg : segments){
            for(int i = 0; i < seg.points.size(); i++){
                if(!this.contains(seg.get(i))) this.add(seg.get(i));
            }
        }
        if(this.size() > 0) this.remove(0);
        if(this.size() > 0) this.remove(this.size() - 1);
    }

    public Vertex end(){
        return this.size() > 0 ? this.get(this.size() - 1) : this.start;
    }
    public void createFullPath(){
        fullPath.clear();
        fullPath.add(start);
        fullPath.addAll(this);
        fullPath.add(target);
    }

    public void injectPoints(){
        ArrayList<Vertex> newPoints = new ArrayList<>();
        // Create an ArrayList of Edges from the path
        for (PathSegment seg : segments) {
            if (seg.corner) continue;
            Vertex startPoint = seg.get(0);
            Vertex endPoint = seg.get(1);
            Vector vector = endPoint.createVector(startPoint);
            double mag = vector.magnitude();
            double numPoints = Math.round(mag / pointSpacing);
            vector = vector.normalize().scale(pointSpacing);
            for (int i = 0; i < numPoints; i++) {
                newPoints.add(startPoint.moveByVector(vector.scale(i)));
            }
            newPoints.add(endPoint);
            seg.replace(newPoints);
        }
    }

    // Pretends it is a trajectory so it can be visualized in advantage scope
    public void sendToSmartDashboard(){
        ArrayList<Double> arr = new ArrayList<>();
        for(Vertex v : this){
            arr.add(v.x);
            arr.add(v.y);
            arr.add(0.0);
        }
        SmartDashboard.putNumberArray("Field/PathSmooth", arr.toArray(new Double[0]));
    }

    public Rotation2d getFinalRot(){
        Vertex end = end();
        if(end == null) return HDriveSuper.poseEstimator.getEstimatedPosition().getRotation();
        return new Rotation2d(Math.atan2(target.y - end.y, target.x - end.x));
    }

    public static Path fromDoubleArray(double[] arr){
        Vertex start = new Vertex(arr[0], arr[1], Rotation2d.fromDegrees(arr[2]));
        Vertex unsnappedTarget = new Vertex(arr[arr.length - 6], arr[arr.length -5], Rotation2d.fromDegrees(arr[arr.length -4]));
        Vertex target = new Vertex(arr[arr.length - 3], arr[arr.length -2], Rotation2d.fromDegrees(arr[arr.length -1]));
        Path path = new Path(start,target);
        for(int i = 3; i < arr.length - 6; i +=3){
            path.add(new Vertex(arr[i], arr[i + 1], Rotation2d.fromDegrees(arr[i + 2])));
        }
        path.unsnappedTarget = unsnappedTarget;
        return path;
    }
}