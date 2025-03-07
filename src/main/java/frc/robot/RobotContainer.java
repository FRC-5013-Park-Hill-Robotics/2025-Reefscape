// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.PoseCommandFactory;
import frc.robot.commands.goToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final PoseCommandFactory mPoseCommandFactory = new PoseCommandFactory(this);

    public static RobotContainer instance;
    //private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController mJoystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain mDrivetrain = TunerConstants.createDrivetrain();

    //public final Elevator mElevator = new Elevator();

    private Field2d m_field = new Field2d();

    //Front false by default for auto, enabled in teleop
    //public final LimeLight frontLimeLight = new LimeLight("limelight-front", true);
    //public final LimeLight backLimeLight = new LimeLight("limelight-back", true);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        instance = this;
        configureBindings();

        SmartDashboard.putData("Field", m_field);

        SmartDashboard.clearPersistent("Auto Chooser");
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser(); 

        SmartDashboard.putData("Auto Chooser", autoChooser);

        mDrivetrain.resetPose(new Pose2d(0,0, new Rotation2d(0)));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        mDrivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            mDrivetrain.applyRequest(() ->
                drive.withVelocityX(-mJoystick.getLeftY() * MaxSpeed * mJoystick.getRightTriggerAxis()) // Drive forward with negative Y (forward)
                    .withVelocityY(-mJoystick.getLeftX() * MaxSpeed * mJoystick.getRightTriggerAxis()) // Drive left with negative X (left)
                    .withRotationalRate(-mJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        mJoystick.a().whileTrue(mDrivetrain.applyRequest(() -> brake));
        mJoystick.b().whileTrue(mDrivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-mJoystick.getLeftY(), -mJoystick.getLeftX()))
        ));

        //mJoystick.x().onTrue(getAutonomousCommand().onlyWhile(new InstantCommand));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // mJoystick.back().and(mJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // mJoystick.back().and(mJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // mJoystick.start().and(mJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // mJoystick.start().and(mJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //mJoystick.leftBumper().whileTrue(mPoseCommandFactory.goToClosestReefPose(true));
        //mJoystick.rightBumper().whileTrue(mPoseCommandFactory.goToClosestReefPose(true));
        mJoystick.leftBumper().whileTrue(new goToPose(new Pose2d(0,0,new Rotation2d(Math.toRadians(0)))));
        mJoystick.rightBumper().whileTrue(new goToPose(new Pose2d(1,1,new Rotation2d(Math.toRadians(180)))));

        // reset the field-centric heading on left bumper press
        mJoystick.back().onTrue(mDrivetrain.runOnce(() -> mDrivetrain.seedFieldCentric()));

        /*mJoystick.a().onFalse(mElevator.setPosC(1));
        mJoystick.b().onFalse(mElevator.setPosC(-1));
        mJoystick.y().onFalse(mElevator.setPosC(-60));
        mJoystick.x().onFalse(mElevator.setPosC(-80));

        mJoystick.povUp().onTrue(mElevator.incrementPosC(-10));
        mJoystick.povDown().onTrue(mElevator.incrementPosC(10));*/

        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //return Commands.print("No autonomous command configured");
        return autoChooser.getSelected();
    }

    public void updateField(){
        Pose2d i = mDrivetrain.getState().Pose;
        m_field.setRobotPose(i);

        SmartDashboard.putNumber("BotX", mDrivetrain.getState().Pose.getX());
        SmartDashboard.putNumber("BotY", mDrivetrain.getState().Pose.getY());
        SmartDashboard.putNumber("BotH", mDrivetrain.getState().Pose.getRotation().getDegrees());
    }
    public static RobotContainer getInstance(){
		return instance;
	}

    public CommandSwerveDrivetrain getDrivetrain(){
        return mDrivetrain;
    }

    // public LimeLight getFrontLimeLight(){
    //     return frontLimeLight;
    // }

    // public LimeLight getBackLimeLight(){
    //     return backLimeLight;
    // }
}
