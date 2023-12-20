package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DetectionHandler extends SubsystemBase {
    public double[] detections;
    // double array of zeros
    double[] empty = {0};

    public DetectionHandler() {
    }

    // Handle ML detection & messaging
    @Override
    public void periodic() {
        //detections = SmartDashboard.getNumberArray("Detections", );
        // TODO get these values from detections NT4 entries
        double X = 0;
        double Y = 0;
        /*double[] objBotRelativePosition = BBoxRangeEstimator(Constants.VisionConstants.DetectorConstants.CamHeight,
            VisionConstants.kDT,
            VisionConstants.focalF,
            X,
            Y);*/


    }
}