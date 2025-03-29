// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.subsystems.LimeLight;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  String autoName;
  String newAutoName;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchNumber());
    m_robotContainer.updateField(); //For updating the smartdashboard field display
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // newAutoName = RobotContainer.getInstance().getAutonomousCommand().getName();
    // if (autoName != newAutoName) {
    //   autoName = newAutoName;
    //   if (AutoBuilder.getAllAutoNames().contains(autoName)) {
    //     System.out.println("Displaying " + autoName);
    //     List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
    //     List<Pose2d> poses = new ArrayList<>();
    //     for (PathPlannerPath path : pathPlannerPaths) {
    //         poses.addAll(path.getAllPathPoints().stream().map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d())).collect(Collectors.toList()));
    //         // path.getAllPathPoints().stream().map(point -> new Pose2d(point.position, 0)).collect(Collectors.toList());
    //     }
    //     RobotContainer.getInstance().getField().getObject("path").setPoses(poses);
    //     //PathPlannerTrajectory pathPlannerTrajectory = new PathPlannerTrajectory(pathPlannerTrajectoryStates);
    //     //RobotContainer.getInstance().getField().getObject("trajectory").setTrajectory((Trajectory) pathPlannerTrajectory);
    //     //PathPlannerTrajectory pathPlannerTrajectory = new PathPlannerTrajectory(pathPlannerPaths, getCurrentRobotChassisSpeeds(), m_fieldRelativeOffset);
    //   }
    // }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //checkUpdateAlliance();
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    //checkUpdateAlliance();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  // private void checkUpdateAlliance() {
  //   Optional<Alliance> alliance = DriverStation.getAlliance();
  //   if (DriverStation.isDSAttached() && alliance.isPresent()) {
  //     LimeLight frontLL = m_robotContainer.getFrontLimeLight();
  //     LimeLight backLL = m_robotContainer.getBackLimeLight();
  //     frontLL.setAlliance(alliance.get());
  //     backLL.setAlliance(alliance.get());
  //     RobotContainer.setAlliance(alliance.get());
  //   }
  // }
}
