package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper1 {
    private PhotonCamera photonCamera1;
    // private PhotonCamera photonCamera2;
    // private PhotonCamera photonCamera3;
    // private PhotonCamera photonCamera4;
    private PhotonPoseEstimator photonPoseEstimator1;
    // private PhotonPoseEstimator photonPoseEstimator2;
    // private PhotonPoseEstimator photonPoseEstimator3;
    // private PhotonPoseEstimator photonPoseEstimator4;


    public PhotonCameraWrapper1() {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        photonCamera1 = new PhotonCamera(VisionConstants.camera1Name);
        // photonCamera2 = new PhotonCamera(VisionConstants.camera2Name);
        // photonCamera3 = new PhotonCamera(VisionConstants.camera3Name);
        // photonCamera4 = new PhotonCamera(VisionConstants.camera4Name);
        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            photonPoseEstimator1 =
                new PhotonPoseEstimator(
                    fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera1, VisionConstants.robotToCam1);
            photonPoseEstimator1.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator1 = null;
        }
    }

    /**
     * @param EstimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     * the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose1(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator1 == null) {
            return Optional.empty();
        }
        SmartDashboard.putNumber("cam yaw", photonCamera1.getLatestResult().getBestTarget().getBestCameraToTarget().getX());

        double[] pose = {prevEstimatedRobotPose.getX(), prevEstimatedRobotPose.getY(), prevEstimatedRobotPose.getRotation().getDegrees()};
        SmartDashboard.putNumberArray("prevPose", pose);
        photonPoseEstimator1.setReferencePose(prevEstimatedRobotPose);
        Pose3d photonRefPose = photonPoseEstimator1.getReferencePose();
        double[] photonPose = {photonRefPose.getX(), photonRefPose.getY(), photonRefPose.getRotation().getY()};
        SmartDashboard.putNumberArray("photonRefPose", photonPose);

        return photonPoseEstimator1.update();
    }
}