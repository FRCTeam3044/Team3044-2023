package frc.robot.subsystems_sim;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.packages.util.LogUtil;
import frc.packages.vision.estimation.CameraProperties;
import frc.packages.vision.estimation.VisionEstimation;
import frc.packages.vision.sim.PhotonCamera;
import frc.packages.vision.sim.PhotonCameraSim;
import frc.packages.vision.sim.SimVisionSystem;
import frc.robot.supers.HDriveSuper;

public class MayTagsSim {

    private final Field2d field;
    private AprilTagFieldLayout tagLayout;

    private final NetworkTableInstance instance;
    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private final PhotonCamera camera3;
    private final PhotonCamera camera4;
    private final List<PhotonCamera> cameras;
    private final List<PhotonPipelineResult> lastResults;
    private final SimVisionSystem visionSim;

    public MayTagsSim() {
        instance = NetworkTableInstance.getDefault();

        camera1 = new PhotonCamera(instance, "front");
        camera2 = new PhotonCamera(instance, "back");
        camera3 = new PhotonCamera(instance, "left");
        camera4 = new PhotonCamera(instance, "right");
        cameras = List.of(camera1, camera2, camera3, camera4);
        lastResults = new ArrayList<>(cameras.size());
        cameras.forEach(c -> lastResults.add(new PhotonPipelineResult()));

        visionSim = new SimVisionSystem("main");
        visionSim.addCamera(
            new PhotonCameraSim(
                camera1,
                CameraProperties.PI4_PICAM2_480p
            ),
            new Transform3d( // robot to camera
                new Translation3d(
                    Units.inchesToMeters(10),
                    0,
                    Units.inchesToMeters(25)
                ),
                new Rotation3d(
                    0,
                    //camera angle
                    -Math.toRadians(8),
                    Math.toRadians(0)
                )
            )
        );
        visionSim.addCamera(
            new PhotonCameraSim(
                camera2,
                CameraProperties.PI4_PICAM2_480p
            ),
            new Transform3d( // robot to camera
                new Translation3d(
                    Units.inchesToMeters(-10),
                    0,
                    Units.inchesToMeters(25)
                ),
                new Rotation3d(
                    0,
                    //camera angle
                    Math.toRadians(8),
                    Math.toRadians(180)
                )
            )
        );
        visionSim.addCamera(
            new PhotonCameraSim(
                camera3,
                CameraProperties.PI4_PICAM2_480p
            ),
            new Transform3d( // robot to camera
                new Translation3d(
                    0,
                    Units.inchesToMeters(10),
                    Units.inchesToMeters(25)
                ),
                new Rotation3d(
                    0,
                    //camera angle
                    Math.toRadians(8),
                    Math.toRadians(270)
                )
            )
        );
        visionSim.addCamera(
            new PhotonCameraSim(
                camera4,
                CameraProperties.PI4_PICAM2_480p
            ),
            new Transform3d( // robot to camera
                new Translation3d(
                    0,
                    Units.inchesToMeters(-10),
                    Units.inchesToMeters(25)
                ),
                new Rotation3d(
                    0,
                    //camera angle
                    -Math.toRadians(8),
                    Math.toRadians(90)
                )
            )
        );

        try {
            tagLayout = new AprilTagFieldLayout("src/main/deploy/2023-taglayout.json");
        } catch (IOException e) {
            // Drive Station error reporting
            DriverStation.reportError("Fatal: Missing tag layout JSON", e.getStackTrace());

            // Console reporting
            System.out.println("Fatal: Missing tag layout JSON");
            e.printStackTrace();
            System.exit(1);
        }

        visionSim.addVisionTargets(tagLayout);
        field = visionSim.getDebugField();
    }

    //----- Simulation
    public void simulationPeriodic() {
        visionSim.update(HDriveSim.getTruePose());
        field.getObject("Noisy Robot").setPose(HDriveSim.getTruePose());

        var visCorners = new ArrayList<TargetCorner>();
        var knownVisTags = new ArrayList<AprilTag>();
        var relVisTagsPnP = new ArrayList<AprilTag>();
        var relVisTagsTrig = new ArrayList<AprilTag>();

        var bestPoses = new ArrayList<Pose2d>();
        var altPoses = new ArrayList<Pose2d>();
        var testPoses = new ArrayList<Pose2d>();

        boolean updated = false;
        for (int i = 0; i < cameras.size(); i++) {
            var camera = cameras.get(i);
            var cameraSim = visionSim.getCameraSim(camera.getName());
            var robotToCamera = visionSim.getRobotToCamera(cameraSim);
            var result = camera.getLatestResult();

            if (result.getTimestampSeconds() == lastResults.get(i).getTimestampSeconds()) continue;
            else {
                lastResults.set(i, result);
                updated = true;
            }

            for (var target : result.getTargets()) {
                if (target.getFiducialId() <= 1 && target.getFiducialId() >= 8) continue;
                if (target.getPoseAmbiguity() < 0.1 && target.getPoseAmbiguity() != -1) continue;
                if (target.getFiducialId() == -1) continue;
                visCorners.addAll(target.getDetectedCorners());
                Pose3d tagPose = tagLayout.getTagPose(target.getFiducialId()).get();
                // actual layout poses of visible tags
                knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));
                Transform3d camToBest = target.getBestCameraToTarget();
                // tags estimated relative to robot
                relVisTagsPnP.add(new AprilTag(
                    target.getFiducialId(),
                    new Pose3d().plus(robotToCamera).plus(camToBest)
                ));
                var undistortedTarget = cameraSim.prop.undistort2dTarget(target);
                relVisTagsTrig.addAll(VisionEstimation.estimateTagsTrig(
                    robotToCamera,
                    List.of(undistortedTarget),
                    tagLayout
                ));
                Transform3d camToAlt = target.getAlternateCameraToTarget();

                var bestPose = tagPose
                    .transformBy(camToBest.inverse())
                    .transformBy(robotToCamera.inverse())
                    .toPose2d();

                bestPoses.add(bestPose);

                altPoses.add(
                    tagPose
                        .transformBy(camToAlt.inverse())
                        .transformBy(robotToCamera.inverse())
                        .toPose2d()
                );
                testPoses.add(
                    new Pose3d(HDriveSim.getTruePose())
                        .plus(new Transform3d(new Pose3d(), relVisTagsTrig.get(relVisTagsTrig.size() - 1).pose))
                        .toPose2d()
                );
            }

            // multi-target solvePNP
            if (result.getTargets().size() > 1) {
                var pnpResults = VisionEstimation.estimateCamPosePNP(
                    cameraSim.prop,
                    visCorners,
                    knownVisTags
                );
                var best = new Pose3d()
                    .plus(pnpResults.best) // field-to-camera
                    .plus(robotToCamera.inverse()); // field-to-robot
                var alt = new Pose3d()
                    .plus(pnpResults.alt) // field-to-camera
                    .plus(robotToCamera.inverse()); // field-to-robot
                bestPoses.clear();
                altPoses.clear();
                bestPoses.add(best.toPose2d());
                altPoses.add(alt.toPose2d());
            }
        }
        // multi-target SVD
        if (knownVisTags.size() > 0) {
            var estTrf = VisionEstimation.estimateTransformLS(
                relVisTagsTrig,
                knownVisTags,
                false
            );
            var estRobotPose = estTrf.trf.apply(new Pose3d());
            testPoses.add(estRobotPose.toPose2d());
            SmartDashboard.putNumberArray(
                "VisionEstRobotPose3d",
                LogUtil.toPoseArray3d(estRobotPose)
            );
        }
        if (updated) {
            Translation2d temp_translation = new Translation2d();
            Rotation2d temp_rotation = new Rotation2d();
            for (Pose2d pose : bestPoses) {
                temp_translation = temp_translation.plus(pose.getTranslation());
                temp_rotation = temp_rotation.plus(pose.getRotation());
            }
            if (bestPoses.size() != 0) {
                var translation = temp_translation.div(bestPoses.size());
                var rotation = temp_rotation.div(bestPoses.size());
                if (Math.abs(translation.getNorm() - HDriveSim.getTruePose().getTranslation().getNorm()) < 0.1 && Math.abs(rotation.getDegrees() - HDriveSim.getTruePose().getRotation().getDegrees()) < 5) {
                    Pose2d betterresult = new Pose2d(translation, rotation);
                    HDriveSuper.poseEstimator.addVisionMeasurement(betterresult, edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
                }
            }
            field.getObject("bestPoses").setPose(HDriveSuper.poseEstimator.getEstimatedPosition());
        }
    }
}
