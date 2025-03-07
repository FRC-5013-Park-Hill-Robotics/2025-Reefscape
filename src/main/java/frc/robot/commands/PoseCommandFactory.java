// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.ProcessBuilder.Redirect;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldPositions;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class PoseCommandFactory {

    private CommandSwerveDrivetrain m_drivetrain;

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final PIDController ControllerX = new PIDController(10, 0, 0);
    private final PIDController ControllerY = new PIDController(10, 0, 0);
    private final PIDController ControllerH = new PIDController(10, 0, 0);

    public PoseCommandFactory(RobotContainer robotContainer) {
        ControllerH.enableContinuousInput(0, 360);
        //m_drivetrain = robotContainer.getDrivetrain();
    }

    public Command goToPose(Pose2d target){
        // // Since we are using a holonomic drivetrain, the rotation component of this pose
        // // represents the goal holonomic rotation

        // // Create the constraints to use while pathfinding
        // PathConstraints constraints = new PathConstraints(
        //         //Max was 3...
        //         3.0, 4.0,
        //         Units.degreesToRadians(540), Units.degreesToRadians(720));

        // SmartDashboard.clearPersistent("TargetX");
        // SmartDashboard.clearPersistent("TargetY");
        // SmartDashboard.clearPersistent("TargetH");
        // SmartDashboard.putNumber("TargetX", target.getX());
        // SmartDashboard.putNumber("TargetY", target.getY());
        // SmartDashboard.putNumber("TargetH", target.getRotation().getDegrees());

        // // Since AutoBuilder is configured, we can use it to build pathfinding commands
        // return AutoBuilder.pathfindToPose(
        //         target,
        //         constraints,
        //         0.0 // Goal end velocity in meters/sec
        // );
        double ErrorX = m_drivetrain.getState().Pose.getX() - target.getX();
        double ErrorY = m_drivetrain.getState().Pose.getY() - target.getY();
        double ErrorH = m_drivetrain.getState().Pose.getRotation().getDegrees() - target.getRotation().getDegrees();
        
        double OutputX = ControllerX.calculate(ErrorX);
        double OutputY = ControllerY.calculate(ErrorY);
        double OutputH = ControllerH.calculate(ErrorH);

        return m_drivetrain.applyRequest(() ->
                drive.withVelocityX(0 * OutputX * DriveConstants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(0 * OutputY * DriveConstants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(OutputH * DriveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            );
    }

    public Command goToClosestReefPose(boolean left){
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d goalPose = new Pose2d();
        if(left){
            goalPose = m_drivetrain.getState().Pose.nearest(FieldPositions.Left);
        }
        else{
            goalPose = m_drivetrain.getState().Pose.nearest(FieldPositions.Right);
        }

        return goToPose(goalPose);
    }

}
