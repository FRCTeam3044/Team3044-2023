package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper2 {
    // private PhotonCamera photonCamera1;
    private PhotonCamera photonCamera2;
    // private PhotonCamera photonCamera3;
    // private PhotonCamera photonCamera4;
    // private PhotonPoseEstimator photonPoseEstimator1;
    private PhotonPoseEstimator photonPoseEstimator2;
    // private PhotonPoseEstimator photonPoseEstimator3;
    // private PhotonPoseEstimator photonPoseEstimator4;


    public PhotonCameraWrapper2() {
        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        // photonCamera1 = new PhotonCamera(VisionConstants.camera1Name);
        photonCamera2 = new PhotonCamera(VisionConstants.camera2Name);
        // photonCamera = new PhotonCamera(VisionConstants.camera3Name);
        // photonCamera = new PhotonCamera(VisionConstants.camera4Name);
        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
            // Create pose estimator
            photonPoseEstimator2 =
                new PhotonPoseEstimator(
                    fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera2, VisionConstants.robotToCam2);
            photonPoseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            photonPoseEstimator2 = null;
        }
    }

    /**
     * @param EstimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     * the estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose2(Pose2d prevEstimatedRobotPose) {
        if (photonPoseEstimator2 == null) {
            return Optional.empty();
        }
        photonPoseEstimator2.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator2.update();
    }
}