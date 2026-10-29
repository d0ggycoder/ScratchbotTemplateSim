// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Components.ComponentHandler;
import frc.robot.Components.PositionComponent;
import frc.robot.PremadeDrivetrains.SwerveDrivetrain.Drivetrain;
import frc.robot.PremadeDrivetrains.SwerveDrivetrain.GoToPoint;
import frc.robot.PremadeDrivetrains.SwerveDrivetrain.DriveVelocity;
import frc.robot.Scratchbot.Scratchbot;

public class Robot extends LoggedRobot {
    public static Robot instance;
    public static XboxController controller = new XboxController(0);

    @Override
    public void robotInit(){ 
        //initialize the instance variable
        instance = this;

        Logger.recordMetadata("ProjectName", "ScratchBot");
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("GitLastCommitTime", BuildConstants.GIT_DATE);

        boolean replay = false;
        
        if(isReal()){
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            if(replay){
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
            } else {
                Logger.addDataReceiver(new NT4Publisher());
            }
        }
        
        Logger.start();

        ComponentHandler.initialize();
        Drivetrain.getInstance().setDefaultCommand(new DriveVelocity(
            controller::getLeftX,
            controller::getLeftY,
            controller::getRightX
        ));
    }

    @Override
    public void robotPeriodic() {
        ComponentHandler.periodic();
        CommandScheduler.getInstance().run();
    }
    @Override
    public void teleopInit() {
        /*TODO: set your team's teleop command as the default command for 
        the subsystem if you want rookies to be able to reposition the robot*/
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void autonomousInit() {
        CommandScheduler.getInstance().schedule(new GoToPoint(
            new Pose2d(2,2,Rotation2d.kZero),
            PositionComponent.getInstance()::getPose2d,
            Drivetrain.getInstance()::getChassisSpeeds
        ));
    }

    @Override
    public void autonomousPeriodic() {
    }
    @Override
    public void testInit() {
    //Run Lab
    Scratchbot.initializeLab();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
    }
}
