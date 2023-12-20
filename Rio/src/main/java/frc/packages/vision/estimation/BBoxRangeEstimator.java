package frc.packages.vision.estimation;

public class BBoxRangeEstimator{

    private final double cameraHeight;
    //height of camera off the ground

    private final double callibrationValue;
    //value for camera callibration constant

    private final double focalF;
    //focal length

    private final double centerX;
    //x value of center pixel

    private final double centerY;
    //y value of center pixel

    /**
     * @param h The height of the camera from the ground
     * @param k The camera calibration value
     * @param F the focal F of the camera
     * @param Cix the center X of the BBox
     * @param Ciy the center Y of the BBox
     */
    public BBoxRangeEstimator(double h, double k, double F, double Cix, double Ciy){
        cameraHeight = h;
        callibrationValue = k;
        focalF = F;
        centerX = Cix;
        centerY = Ciy;
    }


    public double[] findRangeFromBBox(double topBBox, double bottomBBox, double leftBBox, double rightBBox){
        
        double cubeCenterHeightFromGround = (topBBox - bottomBBox)/2;
        //center of cube from top y value and bottom y value

        double cubeCenterLeftRightDistanceFromBBox = (leftBBox - rightBBox)/2;
        //center of cube from left x value and bottom x value

        double triangeHeight = cameraHeight - cubeCenterHeightFromGround;
        //height of the triangle from camera to the height of the center of the cube

        double cameraVerticalAngleCameraCenterToCubeCenter = Math.atan( ( (centerY - cubeCenterHeightFromGround) * callibrationValue) / focalF);
        //the angle between the center pixel y value and the center point of the cube

        double cameraAngleGroundToCubeCenter = 90 - cameraVerticalAngleCameraCenterToCubeCenter;
        //the opposite angle of cameraAngleCameraCenterToCubeCenter that is inside the triangle

        //double angleBotToCamera = 180 - 90 - cameraAngleGroundToCubeCenter;
        //the 3rd angle in the triangle, since one angle is 90 and the other is known
        //Idk if this useful, but David said I needed this value

        double CubeToBotGroundDistance = Math.tan(cameraAngleGroundToCubeCenter) * triangeHeight;
        //ground distance between the robot and the cube so long as the cube
        //is perfectly centered horizontally

        double cameraHorizontalAngleGroundToCubeCenter = Math.atan( ( (centerX - cubeCenterLeftRightDistanceFromBBox) * callibrationValue ) / focalF);
        //the angle between the center pixel x value and the center point of the cube

        double horizontalShiftFromBot = Math.sin(cameraHorizontalAngleGroundToCubeCenter) * CubeToBotGroundDistance;
        //horizontal distance between the center of the camera and the center of the cube
        //using the law of sines

        double forwardDistanceFromBot = Math.sqrt((Math.pow(CubeToBotGroundDistance, 2) - Math.pow(horizontalShiftFromBot, 2)));
        //forward distance between the bot and the horizontal line that the cube center point is on

        return new double[]{forwardDistanceFromBot, horizontalShiftFromBot};
    }
}

