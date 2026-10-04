package frc.robot.PremadeDrivetrains.SwerveDrivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import frc.robot.IOModels.Motors.MotorIO.MotorConfig;
import frc.robot.IOModels.SwerveModules.MaxSwerveModule;

// TODO: Change/update these values for you team!
public class DrivetrainConstants {
    public static final double steerPositionP = 0.3;
    public static final double steerPositionI = 0;
    public static final double steerPositionD = 0.02;

    public static final double driveVelocityP = 0.08;
    public static final double driveVelocityI = 0;
    public static final double driveVelocityD = 0.04;

    public static final MotorConfig steerMotorConfig = MaxSwerveModule.getDefaultSteerMotorConfig().withPositionPID(
        steerPositionP,
        steerPositionI,
        steerPositionD   
    );
    public static final MotorConfig driveMotorConfig = MaxSwerveModule.getDefaultDriveMotorConfig().withVelocityPID(
        steerPositionP,
        steerPositionI,
        steerPositionD
    );

    public static final int frontLeftSteerMotorID = 1;
    public static final int frontLeftDriveMotorID = 2;
    public static final int frontRightSteerMotorID = 3;
    public static final int frontRightDriveMotorID = 4;
    public static final int backLeftSteerMotorID = 5;
    public static final int backLeftDriveMotorID = 6;
    public static final int backRightSteerMotorID = 7;
    public static final int backRightDriveMotorID = 8;
    

    public static final Translation2d frontLeftLocation = new Translation2d(0.301625, 0.301625);
    public static final Translation2d frontRightLocation = new Translation2d(0.301625, -0.301625);
    public static final Translation2d backLeftLocation = new Translation2d(-0.301625, 0.301625);
    public static final Translation2d backRightLocation = new Translation2d(-0.301625, -0.301625);
    public static final Translation2d[] swerveModulePositions = {frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation};

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        swerveModulePositions
    );

    public static final Mass robotMass = Pounds.of(120);
    public static final Distance bumperWidth = Inches.of(24);
    public static final Distance bumperLength = Inches.of(24); 

    public static final double maxDriveSpeed = 10;
    public static final double maxTurnSpeed = 3*Math.PI;

    public static final double defaultDriveSpeed = 8;
    public static final double defaultTurnSpeed = 2*Math.PI;
}
