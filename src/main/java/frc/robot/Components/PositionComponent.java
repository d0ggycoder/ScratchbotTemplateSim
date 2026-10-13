package frc.robot.Components;

import java.util.function.Supplier;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.Components.LimelightHelpers.PoseEstimate;
import frc.robot.IOModels.Gyros.GyroIO;
import frc.robot.IOModels.Gyros.NavX2Gyro;

/**
 * A singleton class that encapsulates the {@link SwerveDrivePoseEstimator}
 * to give positions from limelight and odometry. (Uses blue origin)
 * @author Darren Ringer
 */
public class PositionComponent{

    //-------------------------------------------Variables--------------------------------------------//
    private static PositionComponent instance;
    private SwerveDrivePoseEstimator poseEstimator;
    private Supplier<SwerveModulePosition[]> swerveModulePositionSupplier; 
    private Supplier<SwerveModuleState[]> swerveModuleStatesSupplier;
    private SwerveDriveKinematics kinematics;
    private GyroIO gyro;
    private Pose2d lastCache;

    //------------------------------------------Core Methods------------------------------------------//

    private PositionComponent(SwerveDriveKinematics kinematics, Supplier<SwerveModulePosition[]> swerveModulePositionsSupplier, 
                              Supplier<SwerveModuleState[]> swerveModuleStatesSupplier, Pose2d initialPose){
        
        
        this.gyro = NavX2Gyro.getDefaultGyro(NavXComType.kMXP_SPI);
        this.lastCache = initialPose;
        this.kinematics = kinematics;
        this.swerveModulePositionSupplier = swerveModulePositionsSupplier;
        this.swerveModuleStatesSupplier = swerveModuleStatesSupplier;
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation(), swerveModulePositionsSupplier.get(), initialPose);
    }

    /**
     * Gets the instance of PositionComponent or throws an error if none exists yet
     * @return Current instance of PositionComponent
     */
    public static PositionComponent getInstance(){
        if(instance == null){
            throw new Error("Cannot get instance of PositionComponent before initialization :(");
        }
        return instance;
    }
    /**
     * Initializes the PositionComponent
     * @param kinematics Swerve kinematics
     * @param swerveModulePositionsSupplier A supplier of the current drivetrain swerve module positions
     * @param swerveModuleStatesSupplier A supplier of the current drivetrain swerve module states
     * @param initialPose Starting Pose
     * @return Instance of PositionComponent
     */
    public static PositionComponent initialize(SwerveDriveKinematics kinematics, Supplier<SwerveModulePosition[]> swerveModulePositionsSupplier, 
                                               Supplier<SwerveModuleState[]> swerveModuleStatesSupplier, Pose2d initialPose){
        instance = new PositionComponent(kinematics, swerveModulePositionsSupplier, swerveModuleStatesSupplier, initialPose);
        return instance;
    }
    /**
     * PositionComponent periodic loop (should be called in ComponentManager.periodic())
     */
    public void periodic(){
        lastCache = poseEstimator.update(gyro.getRotation(), swerveModulePositionSupplier.get());
        PoseEstimate limelightEstimate = LimelightComponent.calcAprilTag();
        if(limelightEstimate != null){
            double[] stddev = LimelightComponent.getStdDev();
            poseEstimator.addVisionMeasurement(limelightEstimate.pose, limelightEstimate.timestampSeconds,
                    VecBuilder.fill(stddev[0], stddev[1], stddev[2]));
        }
    }
    //--------------------------------------------Getters---------------------------------------------//

    /**
     * Fetches the current robot pose
     * @return The current RobotPose
     */
    public Pose2d getPose2d(){
        return lastCache;
    }

    /**
     * Gets rotation reading from gyro
     * @return Gyro's rotation
     */
    public Rotation2d getGyroReading(){
        return gyro.getRotation();
    }

    //--------------------------------------------Setters---------------------------------------------//

    /**
     * Sets the pose
     * @param newPose The new pose
     */
    public void resetPose(Pose2d newPose){
        poseEstimator.resetPose(newPose);
        lastCache = newPose;
    }
    
    /**
     * Rezeros pose
     */
    public void zeroPose(){
        poseEstimator.resetPose(Pose2d.kZero);
        gyro.reset();
        lastCache = Pose2d.kZero;
    }
}