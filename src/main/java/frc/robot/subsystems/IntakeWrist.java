// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.trobot5013lib.CANCoderWrapper;
import frc.robot.trobot5013lib.RevThroughBoreEncoder;
import frc.robot.trobot5013lib.TrobotUtil;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class IntakeWrist extends SubsystemBase {

    private final TalonFX intakeWristMotor = new TalonFX(IntakeConstants.INTAKE_WRIST_MOTOR_CAN_ID);
    private RevThroughBoreEncoder encoder = new RevThroughBoreEncoder(IntakeConstants.ENCODER_DIO_PORT, true,
            IntakeConstants.ENCODER_OFFSET_RADIANS);
    public double setpointRadians = 0;
    private ArmFeedforward feedforward = new ArmFeedforward(
            IntakeConstants.RotationGains.kS,
            IntakeConstants.RotationGains.kG,
            IntakeConstants.RotationGains.kV,
            IntakeConstants.RotationGains.kA);
    private Constraints wristConstraints = new Constraints(IntakeConstants.RotationGains.kMaxSpeed,
            IntakeConstants.RotationGains.kMaxAcceleration);
    private ProfiledPIDController wristController = new ProfiledPIDController(
            IntakeConstants.RotationGains.kP,
            IntakeConstants.RotationGains.kI,
            IntakeConstants.RotationGains.kD,
            wristConstraints);
    private final VoltageOut wristVoltageOut = new VoltageOut(0);
    private WristGoal wristGoalRadians = new ArmRelativeGoal(0);
    private double lastSpeed = 0;
    private double lastTime = 0;

    private Debouncer mDebouncer = new Debouncer(0.1);
    private Timer mTimer = new Timer();

    private boolean stop = true;
    private IntakeRollers m_intakeRollers;

    public abstract class WristGoal { 
        protected double goal;
        public WristGoal(double goal) {
            this.goal = goal;
        }
        abstract double getGoal(); 
        public void incrementAngleBy(double radianChange) {
            this.goal += radianChange;
        }
    }

    public class GroundRelativeGoal extends WristGoal {
        public GroundRelativeGoal(double goal) {
            super(goal);
        }
        double getGoal() {
            return getGroundRelativeWristPositionRadians(goal);
        }
    }

    public class ArmRelativeGoal extends WristGoal {
        public ArmRelativeGoal(double goal) {
            super(goal);
        }
        double getGoal() {
            return goal;
        }
    }

    /** Creates a new IntakeShoulder. */
    public IntakeWrist(IntakeRollers intakeRollers) {
        super();
        this.m_intakeRollers = intakeRollers;

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeWristMotor.getConfigurator().apply(config);
        wristController.setTolerance(IntakeConstants.RotationGains.kPositionTolerance.getRadians(), 0.01);
        wristController.enableContinuousInput(0, 2 * Math.PI);

        mTimer.reset();
    }

    public double getAngle() {
        double value = encoder.getAngle().getRadians();
        if (value < 0) {
            value += 2 * Math.PI;
        }
        return value;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeAngle", Math.toDegrees(getAngle()));
        SmartDashboard.putNumber("IntakeDeployAngle", Math.toDegrees(getGroundRelativeWristPositionRadians(IntakeConstants.DEPLOY_SETPOINT_TO_GROUND)));
        if (this.stop == true) {
            intakeWristMotor.setControl(wristVoltageOut.withOutput(0));
        } else {
            if (wristGoalRadians.getGoal() != wristController.getGoal().position) {
                wristController.setGoal(wristGoalRadians.getGoal());
            }
            double pidVal = wristController.calculate(getAngle());
            SmartDashboard.putNumber("pidValue", pidVal);
            State setpoint = wristController.getSetpoint();
            double groundRelativeSetpointRadians = getGroundRelativeWristPositionRadians(setpoint.position);
            double acceleration = (wristController.getSetpoint().velocity - lastSpeed)
                    / (Timer.getFPGATimestamp() - lastTime);
            double feedforwardVal = feedforward.calculate(getGroundRelativeWristPositionRadians(),
                    wristController.getSetpoint().velocity, acceleration);
            SmartDashboard.putNumber("feedforwardVal", feedforwardVal);

            //if intake is at 0 it is resting on launcher and does not need to have any
            //feed forward or pid adjustemnt.  Save motor stall.
            if (wristGoalRadians.getGoal() == 0 && atGoal()){
                pidVal = 0;
                feedforwardVal = 0;
            }

            intakeWristMotor
                    .setControl(wristVoltageOut.withOutput(MathUtil.clamp(pidVal + feedforwardVal, -12.0, 12.0)));
            lastSpeed = wristController.getSetpoint().velocity;
            lastTime = Timer.getFPGATimestamp();
        }
        SmartDashboard.putBoolean("Intake at goal", atGoal());
        SmartDashboard.putNumber("Ground Angle", Math.toDegrees(getGroundRelativeWristPositionRadians()));
        SmartDashboard.putNumber("Wrist Position", encoder.getAngle().getDegrees());
        SmartDashboard.putNumber("Wrist  goal", Math.toDegrees(wristGoalRadians.getGoal()));
        SmartDashboard.putBoolean("HasGamePieceAndDown", hasGamePieceAndDown());
        
    }

    public void deploy() {
        this.stop = false;
        double goal = IntakeConstants.DEPLOY_SETPOINT_TO_GROUND;
        setWristGoalRadians(new GroundRelativeGoal(goal));
    }

    public void amp() {
        setWristGoalRadians(new GroundRelativeGoal(IntakeConstants.AMP_ANGLE_GROUND));
    }

    public Command ampCommand() {
        Command result = run(this::amp).until(this::atAmp);
        //result.addRequirements(getLauncherShoulder());
        return result;
    }

    public double getGoal() {
        return wristGoalRadians.getGoal();
    }

    public boolean atGoal() {
        double errorBound = Math.PI;
        double m_positionError = MathUtil.inputModulus(wristGoalRadians.getGoal() - getAngle(), -errorBound, errorBound);
        return TrobotUtil.withinTolerance(m_positionError,0.0,IntakeConstants.RotationGains.kPositionTolerance.getRadians());
    }

    public void retract() {
        this.stop = false;
        setWristGoalRadians(new ArmRelativeGoal(IntakeConstants.RETRACT_SETPOINT));
    }

    public void stop() {
        this.stop = true;
    }

    public void setWristGoalRadians(WristGoal goal) {
        wristGoalRadians = goal;
    }

    private LauncherShoulder getLauncherShoulder() {
        return RobotContainer.getInstance().getLauncherShoulder();
    }

    protected double getGroundRelativeWristPositionRadians(double launcherRelativeAngleRadians) {
        return Math.PI - getLauncherShoulder().getShoulderAngleRadians() - launcherRelativeAngleRadians;
    }

    public double getGroundRelativeWristPositionRadians() {
        return getGroundRelativeWristPositionRadians(getAngle());
    }

    public Command deployCommand() {
        Command result = runOnce(this::deploy);
        //result.addRequirements(getLauncherShoulder());
        return result;
    }

    public Command retractCommand() {
        Command result = runOnce(this::retract);
        //result.addRequirements(getLauncherShoulder());
        return result;
    }

    public Command stopCommand() {
        Command result = runOnce(this::stop);
        return result;
    }

    public Command intakeGamePiece() {
        return deployCommand().andThen(m_intakeRollers.intakeGamepieceCommand()).until(this::hasGamePieceAndDown)
                .andThen(retractCommand());
    }

    public boolean hasGamePieceAndDown() {
        boolean value = false;
        if(MathUtil.isNear(getGroundRelativeWristPositionRadians(), IntakeConstants.DEPLOY_SETPOINT_TO_GROUND, Math.toRadians(2))){
            mTimer.start();
        } 
        if (mTimer.get() > 0.4){
            value = mDebouncer.calculate(m_intakeRollers.hasGamePiece());
        }
        return value;
    }

    public Command intakeGamePieceManualCommand() {
        return deployCommand().andThen(m_intakeRollers.takeIn());
    }

    public Command intakeGamePieceManualEndCommand() {
        return m_intakeRollers.stopC().andThen(retractCommand());
    }

    public void incrementAngle(double radianChange) {
        this.wristGoalRadians.incrementAngleBy(radianChange);

    }

    public Command incrementAngleCommand(double radianChange) {
        Command result = runOnce(() -> incrementAngle(radianChange));
        return result;
    }

    public boolean atAmp() {
        return TrobotUtil.withinTolerance(getAngle(), IntakeConstants.AMP_ANGLE_GROUND,
                IntakeConstants.RotationGains.kPositionTolerance.getRadians());
    }
    /*
     * private final MutableMeasure<Voltage> m_appliedVoltage =
     * mutable(Volts.of(0));
     * // Mutable holder for unit-safe linear distance values, persisted to avoid
     * // reallocation.
     * private final MutableMeasure<Angle> m_rotation = mutable(Radians.of(0));
     * // Mutable holder for unit-safe linear velocity values, persisted to avoid
     * // reallocation.
     * private final MutableMeasure<Velocity<Angle>> m_velocity =
     * mutable(RadiansPerSecond.of(0));
     * private final VoltageOut m_voltageOut = new VoltageOut(0);
     * private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
     * new SysIdRoutine.Config( Volts.of(0.75).per(Seconds.of(1)), Volts.of(4),
     * null,null),
     * new SysIdRoutine.Mechanism(
     * // Tell SysId how to plumb the driving voltage to the motors.
     * (Measure<Voltage> volts) -> {
     * intakeWristMotor.setControl(m_voltageOut.withOutput(volts.in(Volts)));
     * },
     * // Tell SysId how to record a frame of data for each motor on the mechanism
     * // being
     * // characterized.
     * log -> {
     * // Record a frame for the wrist motor.
     * log.motor("wrist")
     * .voltage(
     * m_appliedVoltage.mut_replace(intakeWristMotor.get() *
     * RobotController.getBatteryVoltage()
     * , Volts))
     * .angularPosition(m_rotation.mut_replace(getGroundRelativeWristPositionRadians
     * (), Radians))
     * .angularVelocity(
     * m_velocity.mut_replace(encoder.getVelocityRadians(), RadiansPerSecond));
     * 
     * },
     * // Tell SysId to make generated commands require this subsystem, suffix test
     * // state in
     * // WPILog with this subsystem's name ("IntakeWrist")
     * this));
     * 
     * public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
     * return m_sysIdRoutine.quasistatic(direction);
     * }
     * 
     * public Command sysIdDynamic(SysIdRoutine.Direction direction) {
     * return m_sysIdRoutine.dynamic(direction);
     * }
     */
}
