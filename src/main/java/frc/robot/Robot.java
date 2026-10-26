// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Components.ComponentHandler;
import frc.robot.Components.PositionComponent;
import frc.robot.PremadeDrivetrains.SwerveDrivetrain.Drivetrain;
import frc.robot.PremadeDrivetrains.SwerveDrivetrain.GoToPoint;
import frc.robot.Scratchbot.Scratchbot;

public class Robot extends TimedRobot {
  public static Robot instance;
  public static XboxController controller = new XboxController(0);

  @Override
  public void robotInit() {
    ComponentHandler.initialize();
    //initialize the instance variable
    instance = this;
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
    // System.out.println(PositionComponent.getInstance().getPose2d());
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
}
