package frc.robot.Components;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.PremadeDrivetrains.SwerveDrivetrain.Drivetrain;
import frc.robot.PremadeDrivetrains.SwerveDrivetrain.DrivetrainConstants;

public class ComponentHandler {
    public static void initialize(){
        PositionComponent.initialize(
            DrivetrainConstants.kinematics,
            Drivetrain.getInstance()::getSwerveModulePositions,
            Drivetrain.getInstance()::getSwerveModuleStates, 
            Pose2d.kZero
        );
    }

    public static void periodic(){
        PositionComponent.getInstance().periodic();
        Logger.recordOutput("Robot/RobotPose",PositionComponent.getInstance().getPose2d());
    }
}
