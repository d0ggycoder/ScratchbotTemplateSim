package frc.robot.PremadeDrivetrains.SwerveDrivetrain;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.IOModels.Gyros.NavX2Gyro;
import frc.robot.IOModels.SwerveModules.MaxSwerveModule;
import frc.robot.IOModels.SwerveModules.SwerveModuleIO;

public class Drivetrain extends SubsystemBase{
    private static Drivetrain instance;

    private final SwerveDriveKinematics kinematics;
    private final SwerveModuleIO[] swerveModules;
    private SwerveDriveSimulation sim;
    
    private Drivetrain(SwerveDriveKinematics kinematics, SwerveModuleIO[] swerveModules){
        this.kinematics = kinematics;
        this.swerveModules = swerveModules;
    }

    public static Drivetrain getInstance(){
        if(instance == null){
            SwerveModuleIO[] swerveModules = {
                new MaxSwerveModule(
                    DrivetrainConstants.frontLeftDriveMotorID,DrivetrainConstants.driveMotorConfig,
                    DrivetrainConstants.frontLeftSteerMotorID,DrivetrainConstants.steerMotorConfig
                ),new MaxSwerveModule(
                    DrivetrainConstants.frontRightDriveMotorID,DrivetrainConstants.driveMotorConfig,
                    DrivetrainConstants.frontRightSteerMotorID,DrivetrainConstants.steerMotorConfig
                ),new MaxSwerveModule(
                    DrivetrainConstants.backLeftDriveMotorID,DrivetrainConstants.driveMotorConfig,
                    DrivetrainConstants.backLeftSteerMotorID,DrivetrainConstants.steerMotorConfig
                ),new MaxSwerveModule(
                    DrivetrainConstants.backRightDriveMotorID,DrivetrainConstants.driveMotorConfig,
                    DrivetrainConstants.backRightSteerMotorID,DrivetrainConstants.steerMotorConfig
                )
            };
            instance = new Drivetrain(DrivetrainConstants.kinematics, swerveModules);
            if(Robot.isSimulation()){
                instance.setupSimulation();
            }
        }
        return instance;
    }

    private void setupSimulation(){
        @SuppressWarnings("unchecked")
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default().withSwerveModules(
            new MaxSwerveModule.SimulatedMaxSwerveModuleGetter(swerveModules[0]),
            new MaxSwerveModule.SimulatedMaxSwerveModuleGetter(swerveModules[1]),
            new MaxSwerveModule.SimulatedMaxSwerveModuleGetter(swerveModules[2]),
            new MaxSwerveModule.SimulatedMaxSwerveModuleGetter(swerveModules[3])
        ).withGyro(new NavX2Gyro.SimualtedNavX2GyroGetter(NavX2Gyro.getDefaultGyro(NavXComType.kMXP_SPI)))
        .withCustomModuleTranslations(DrivetrainConstants.swerveModulePositions)
        .withBumperSize(DrivetrainConstants.bumperLength, DrivetrainConstants.bumperWidth)
        .withRobotMass(DrivetrainConstants.robotMass);
        sim = new SwerveDriveSimulation(config, Pose2d.kZero);
        SimulatedArena.getInstance().addDriveTrainSimulation(sim);
    }

    public void setSwerveStates(double vx, double vy, double vomega, boolean fieldRelative){
        // What to multiply the speeds by in order to keep them in the drive speed range
        double descale = Math.min(1.0,DrivetrainConstants.maxDriveSpeed/Math.sqrt(vx*vx+vy*vy));
        vx*=descale;
        vy*=descale;

        vomega = Math.signum(vomega)*(Math.min(Math.abs(vomega),DrivetrainConstants.maxTurnSpeed));

        ChassisSpeeds requestedSpeeds = 
            fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vomega, null) 
            : new ChassisSpeeds(vx,vy,vomega);
        
        SwerveModuleState[] requestedStates = kinematics.toSwerveModuleStates(requestedSpeeds);
        for(int i=0;i<swerveModules.length;i++){
            swerveModules[i].setSwerveState(requestedStates[i]);
        }
    }

    public SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] ret = new SwerveModulePosition[swerveModules.length];
        for(int i=0;i<swerveModules.length;i++){
            ret[i]=swerveModules[i].getSwerveModulePosition();
        }
        return ret;
    }

    public SwerveModuleState[] getSwerveModuleStates(){
        SwerveModuleState[] ret = new SwerveModuleState[swerveModules.length];
        for(int i=0;i<swerveModules.length;i++){
            ret[i]=swerveModules[i].getSwerveModuleState();
        }
        return ret;
    }
}
