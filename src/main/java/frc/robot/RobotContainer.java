// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.GamepadDrive;
import frc.robot.commands.GoBack;
import frc.robot.commands.PoseCommandFactory;
import frc.robot.commands.goToClosestPose;
import frc.robot.commands.goToPose;
import frc.robot.constants.ElevatorWristSetpoints;
import frc.robot.constants.FieldPositions;
import frc.robot.constants.IntakeConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LimeLight;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public static RobotContainer instance;
    //private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController mDriver = new CommandXboxController(0);
    private final CommandXboxController mOperator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain mDrivetrain = TunerConstants.createDrivetrain();
   
    public final IntakeWrist mIntakeWrist = new IntakeWrist();
    public final IntakeRollers mIntakeRollers = new IntakeRollers();
    public final Elevator mElevator = new Elevator();

    private Field2d m_field = new Field2d();

    //Front false by default for auto, enabled in teleop
    public final LimeLight frontLimeLight = new LimeLight("limelight-front", true);
    public final LimeLight backLimeLight = new LimeLight("limelight-back", true);

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
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // mJoystick.back().and(mJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // mJoystick.back().and(mJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // mJoystick.start().and(mJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // mJoystick.start().and(mJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // mDrivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     mDrivetrain.applyRequest(() ->
        //         drive.withVelocityX(-mJoystick.getLeftY() * MaxSpeed * mJoystick.getRightTriggerAxis()*((mJoystick.getLeftTriggerAxis()>0.5)?0.5:1)) // Drive forward with negative Y (forward)
        //             .withVelocityY(-mJoystick.getLeftX() * MaxSpeed * mJoystick.getRightTriggerAxis()*((mJoystick.getLeftTriggerAxis()>0.5)?0.5:1)) // Drive left with negative X (left)
        //             .withRotationalRate(-mJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        mDrivetrain.setDefaultCommand(new GamepadDrive(mDriver));

        mDriver.leftBumper().whileTrue(new goToClosestPose(FieldPositions.Left));
        mDriver.rightBumper().whileTrue(new goToClosestPose(FieldPositions.Right));

        // reset the field-centric heading on left bumper press
        mDriver.back().onTrue(mDrivetrain.runOnce(() -> mDrivetrain.seedFieldCentric()));

        mDriver.a().onTrue(mIntakeRollers.autoIntakeCoralC())
                            .onFalse(mIntakeRollers.setTargetC(0));
        mDriver.b().onTrue(mIntakeRollers.setTargetC(IntakeConstants.OutakeSpeed))
                            .onFalse(mIntakeRollers.setTargetC(0));
        mDriver.x().onTrue(mIntakeRollers.autoIntakeAlgaeC())
                            .onFalse(mIntakeRollers.setTargetC(0));
        mDriver.y().onTrue(new GoBack(1));
        
        // mJoystick.b().onFalse(mIntakeWrist.setPosC(0));

        mOperator.povUp().onTrue(mElevator.incrementPosC(-2));
        mOperator.povDown().onTrue(mElevator.incrementPosC(2));

        mOperator.povLeft().onTrue(mIntakeWrist.incrementPosC(-2));
        mOperator.povRight().onTrue(mIntakeWrist.incrementPosC(2));
        

        mOperator.back().onTrue(frontLimeLight.setAprilTagViableC(false).alongWith(backLimeLight.setAprilTagViableC(false)));

        mOperator.leftBumper().onTrue(mElevator.setPosC(ElevatorWristSetpoints.L2AE)
                            .alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.L2AW)));
        mOperator.rightBumper().onTrue(mElevator.setPosC(ElevatorWristSetpoints.L3AE)
                            .alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.L3AW)));

        mOperator.a().onTrue(mElevator.setPosC(ElevatorWristSetpoints.IE)
                            .alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.IW)));
        mOperator.x().onTrue(mElevator.setPosC(ElevatorWristSetpoints.L2E)
                            .alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.L2W)));//.onlyIf(mElevator::atPos)));
        mOperator.y().onTrue(mElevator.setPosC(ElevatorWristSetpoints.L3E)
                            .alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.L3W)));
        mOperator.b().onTrue(mElevator.setPosC(ElevatorWristSetpoints.L4E)
                             .alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.L4W)));

        //drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void configureAutonomousCommands() {
        WaitCommand wait5 = new WaitCommand(0.5);
        NamedCommands.registerCommand("Wait0.5", wait5);

        WaitCommand wait2 = new WaitCommand(0.2);
        NamedCommands.registerCommand("Wait0.5", wait2);

        NamedCommands.registerCommand("Load", mElevator.setPosC(ElevatorWristSetpoints.IE).alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.IW)));
        NamedCommands.registerCommand("L2", mElevator.setPosC(ElevatorWristSetpoints.L2E).alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.L2W)));
        NamedCommands.registerCommand("L3", mElevator.setPosC(ElevatorWristSetpoints.L3E).alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.L3W)));
        NamedCommands.registerCommand("L4", mElevator.setPosC(ElevatorWristSetpoints.L4E).alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.L4W)));
        NamedCommands.registerCommand("WaitUntilElevatorAtPos", mElevator.waitUntilAtPosC());
        
        NamedCommands.registerCommand("L2A", mElevator.setPosC(ElevatorWristSetpoints.L2AE).alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.L2AW)));
        NamedCommands.registerCommand("L3A", mElevator.setPosC(ElevatorWristSetpoints.L3AE).alongWith(mIntakeWrist.setPosC(ElevatorWristSetpoints.L3AW)));

        NamedCommands.registerCommand("Intake", mIntakeRollers.autoIntakeCoralC());
        NamedCommands.registerCommand("Outake", mIntakeRollers.setTargetC(IntakeConstants.OutakeSpeed));
        NamedCommands.registerCommand("Stop", mIntakeRollers.setTargetC(0));
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

    public LimeLight getFrontLimeLight(){
        return frontLimeLight;
    }

    public LimeLight getBackLimeLight(){
        return backLimeLight;
    }
}
