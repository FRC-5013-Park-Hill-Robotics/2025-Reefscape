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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.trobot5013lib.CANCoderWrapper;
import frc.robot.trobot5013lib.RevThroughBoreEncoder;
import frc.robot.trobot5013lib.TrobotUtil;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class IntakeWrist extends SubsystemBase {

    private final TalonFX intakeWristMotor = new TalonFX(CANConstants.INTAKE_WRIST_ID, CANConstants.CANBUS_ELEVATOR);
    
    private final RevThroughBoreEncoder encoder = new RevThroughBoreEncoder(CANConstants.WRIST_ENCODER_DIO, true, Math.toRadians(-52));

    public double setpointDegrees = 0;

    private ArmFeedforward feedforward = new ArmFeedforward(
            IntakeConstants.RotationGains.kS,
            IntakeConstants.RotationGains.kG,
            IntakeConstants.RotationGains.kV,
            IntakeConstants.RotationGains.kA);
    private Constraints wristConstraints = new Constraints(IntakeConstants.RotationGains.kMaxSpeed,
            IntakeConstants.RotationGains.kMaxAcceleration);
    private PIDController wristController = new PIDController(
            IntakeConstants.RotationGains.kP,
            IntakeConstants.RotationGains.kI,
            IntakeConstants.RotationGains.kD
            );
    private final VoltageOut wristVoltageOut = new VoltageOut(0);

    //private double lastSpeed = 0;
    //private double lastTime = 0;

    private boolean stop = false;

    /** Creates a new IntakeWrist. */
    public IntakeWrist() {
        super();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeWristMotor.getConfigurator().apply(config);

        wristController.setTolerance(IntakeConstants.RotationGains.kPositionTolerance.getRadians(), 0.01);
        wristController.enableContinuousInput(0, 360);
    }

    @Override
    public void periodic() {
        if (this.stop == true) {
            intakeWristMotor.setControl(wristVoltageOut.withOutput(0));
        } else {
            double output = wristController.calculate(encoder.getAngle().getDegrees(), setpointDegrees);
            SmartDashboard.putNumber("Wrist Setpoint", setpointDegrees);

            //double acceleration = (wristController.getSetpoint().velocity - lastSpeed)
            //        / (Timer.getFPGATimestamp() - lastTime);
            double feedforwardVal = Math.sin(encoder.getAngle().getRadians()) * IntakeConstants.feedforwardMod + IntakeConstants.feedforwardConst;
                    //feedforward.calculate(encoder.getAngle().getRadians(),
                    //wristController.getSetpoint().velocity, acceleration);

            SmartDashboard.putNumber("Wrist Power", output);
            SmartDashboard.putNumber("Wrist FF", feedforwardVal);
            SmartDashboard.putNumber("Wrist Angle", encoder.getAngle().getDegrees());
            SmartDashboard.putNumber("Wrist Combined", MathUtil.clamp(output+feedforwardVal, -IntakeConstants.maxVoltage, IntakeConstants.maxVoltage));
            intakeWristMotor
                    .setControl(wristVoltageOut.withOutput(MathUtil.clamp(output + feedforwardVal, -IntakeConstants.maxVoltage, IntakeConstants.maxVoltage)));
            //lastSpeed = wristController.getSetpoint();
            //lastTime = Timer.getFPGATimestamp();
        }
    }

    public void stop() {
        this.stop = true;
    }

    public double getPosition(){
        return intakeWristMotor.getPosition().getValueAsDouble();
    }

    public void setPosition(double setpoint){
        setpointDegrees = setpoint;
    }

    public void incrementPos(double increment){
        setpointDegrees += increment;
    }

    public Command setPosC(double setpoint){
        Command result = runOnce(() -> setPosition(setpoint));
        return result;
    }

    public Command incrementPosC(double increment){
        Command result = runOnce(() -> incrementPos(increment));
        return result;
    }

    public Boolean getUp(){
        //if(getPosition() < 0 && getPosition() > 0){
        //    return true;
        //} else{
        //    return false;
        //}
        return true;
    }

    
    /*private final MutableMeasure<Voltage> m_appliedVoltage =
    mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Angle> m_rotation = mutable(Radians.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity =
    mutable(RadiansPerSecond.of(0));
    private final VoltageOut m_voltageOut = new VoltageOut(0);
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config( Volts.of(0.75).per(Seconds.of(1)), Volts.of(4),
    null,null),
    new SysIdRoutine.Mechanism(
    // Tell SysId how to plumb the driving voltage to the motors.
    (Measure<Voltage> volts) -> {
    intakeWristMotor.setControl(m_voltageOut.withOutput(volts.in(Volts)));
    },
    // Tell SysId how to record a frame of data for each motor on the mechanism
    // being
    // characterized.
    log -> {
    // Record a frame for the wrist motor.
    log.motor("wrist")
    .voltage(
    m_appliedVoltage.mut_replace(intakeWristMotor.get() *
    RobotController.getBatteryVoltage()
    , Volts))
    .angularPosition(m_rotation.mut_replace(getGroundRelativeWristPositionRadians
    (), Radians))
    .angularVelocity(
    m_velocity.mut_replace(encoder.getVelocityRadians(), RadiansPerSecond));
    
    },
    // Tell SysId to make generated commands require this subsystem, suffix test
    // state in
    // WPILog with this subsystem's name ("IntakeWrist")
    this));
     
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
    }
     
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
    }
    */
}
