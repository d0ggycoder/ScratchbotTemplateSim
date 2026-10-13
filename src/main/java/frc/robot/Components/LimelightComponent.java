package frc.robot.Components;

import frc.robot.Components.LimelightHelpers;
import frc.robot.Components.LimelightHelpers.PoseEstimate;
import frc.robot.Components.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTablesJNI;

/**
 * A class to get positional data from the {@link LimelightHelpers} class with some basic processing
 * @author Darren Ringer
 */
public class LimelightComponent {    
    //-------------------------------------------Constants--------------------------------------------//

    private static final int aprilTagThreshold = 1; // Minimum Apriltags to 
    private static final double aprilTagUncertaintyThreshold = 0.0; // Minimum uncertainty to consider an AprilTag valid (higher = stricter)
    private static final int topicHandle = NetworkTablesJNI.getTopic(NetworkTableInstance.getDefault().getHandle(),"/limelight/stddevs");
    private static final int handle = NetworkTablesJNI.getEntry(topicHandle, NetworkTableType.kDoubleArray.getValue(), NetworkTableType.kDoubleArray.getValueStr());
        
    //-------------------------------------------Last Data--------------------------------------------//
    public static double dist;
    private static Pose2d lPos = new Pose2d();
    private static double[] rawdev;
    private static RawFiducial[] fiducials;

    public static Pose2d getLastAprilTag(){
        return lPos;
    }


    //----------------------------------------Helper functions----------------------------------------//
    public static boolean validMeasurement(){
        int k = aprilTagThreshold;
        for(RawFiducial fiducial:fiducials){
            if(fiducial.ambiguity>=aprilTagUncertaintyThreshold) k--;
        }
        return k<=0;
    }

    public static PoseEstimate calcAprilTag() {
        fiducials = LimelightHelpers.getRawFiducials("");
        PoseEstimate limelightMeasurement;
        if(!validMeasurement()) return null;
        double[] stddevRaw = NetworkTablesJNI.getDoubleArray(handle,new double[12]);

        LimelightHelpers.SetRobotOrientation(null, PositionComponent.getInstance().getPose2d().getRotation().getDegrees(),  0, 0, 0, 0, 0);
        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
        limelightMeasurement.pose = new Pose2d(limelightMeasurement.pose.getX(), limelightMeasurement.pose.getY(), 
                LimelightHelpers.getBotPoseEstimate_wpiBlue(null).pose.getRotation());
        rawdev = new double[]{stddevRaw[6], stddevRaw[7], stddevRaw[5]}; //x, y, yaw (don't trust since it's set using robot)
        
        lPos = limelightMeasurement.pose;
        return limelightMeasurement;
    }

    public static double[] getStdDev(){
        return rawdev;
    }
}