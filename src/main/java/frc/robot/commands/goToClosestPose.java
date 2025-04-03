// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldPositions;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class goToClosestPose extends Command {

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final PIDController ControllerX = DriveConstants.ControllerX;
  private final PIDController ControllerY = DriveConstants.ControllerY;
  private final PIDController ControllerH = DriveConstants.ControllerH;

  private final SlewRateLimiter LimiterX = new SlewRateLimiter(DriveConstants.movementLimitAmount);
  private final SlewRateLimiter LimiterY = new SlewRateLimiter(DriveConstants.movementLimitAmount);
  
  private CommandSwerveDrivetrain m_drivetrain;

  private List<Pose2d> mArrayList;
  private Pose2d mTarget;

  /** Creates a new goToClosestPose. */
  public goToClosestPose(List<Pose2d> target) {
    m_drivetrain = RobotContainer.getInstance().getDrivetrain();
    mArrayList = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTarget = m_drivetrain.getState().Pose.nearest(mArrayList);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("PoseGoalX", mTarget.getX());
    // SmartDashboard.putNumber("PoseGoalY", mTarget.getY());
    // SmartDashboard.putNumber("PoseGoalH", mTarget.getRotation().getDegrees());

    double ErrorX = m_drivetrain.getState().Pose.getX() - mTarget.getX();
    double ErrorY = m_drivetrain.getState().Pose.getY() - mTarget.getY();
    double ErrorH = m_drivetrain.getState().Pose.getRotation().minus(mTarget.getRotation()).getDegrees();

    // SmartDashboard.putNumber("PoseErrorX", ErrorX);
    // SmartDashboard.putNumber("PoseErrorY", ErrorY);
    // SmartDashboard.putNumber("PoseErrorH", ErrorH);

    double i = DriveConstants.xyReduction;
    double a = DriveConstants.goToPoseMaxspeeds;
    double OutputX = i*LimiterX.calculate(MathUtil.clamp(ControllerX.calculate(ErrorX), -DriveConstants.MaxSpeed*a, DriveConstants.MaxSpeed*a));
    double OutputY = i*LimiterY.calculate(MathUtil.clamp(ControllerY.calculate(ErrorY), -DriveConstants.MaxSpeed*a, DriveConstants.MaxSpeed*a));
    double OutputH = MathUtil.clamp(ControllerH.calculate(ErrorH), -DriveConstants.MaxAngularRate*a, DriveConstants.MaxAngularRate*a);

    // SmartDashboard.putNumber("PoseOutputX", OutputX);
    // SmartDashboard.putNumber("PoseOutputY", OutputY);
    // SmartDashboard.putNumber("PoseOutputH", OutputH);

    m_drivetrain.setControl(
      drive.withVelocityX(OutputX)
                  .withVelocityY(OutputY)
                  .withRotationalRate(OutputH)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
