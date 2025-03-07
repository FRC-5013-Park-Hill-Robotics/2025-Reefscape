// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

//import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeConstants;


import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class IntakeRollers extends SubsystemBase {

    // TODO Create motor controller of type TalonFx using the can constants for the id
    private TalonFX intakeRollerMotor = new TalonFX(IntakeConstants.INTAKE_ROLLER_ID, CANConstants.CANBUS_ELEVATOR);
    private double target = 0;
    //private ArmFeedforward m_intakFeedforward = new ArmFeedforward(0, 0, 0);
    private VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0);
    private double ampTarget = IntakeConstants.kAmpOut;

    public IntakeRollers() {
        super();
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Slot0.kP = IntakeConstants.RollerGains.kP;
        config.Slot0.kI = IntakeConstants.RollerGains.kI;
        config.Slot0.kD = IntakeConstants.RollerGains.kD;
        config.Slot0.kS = IntakeConstants.RollerGains.kS;
        config.Slot0.kV = IntakeConstants.RollerGains.kV;
        config.Slot0.kA = IntakeConstants.RollerGains.kA;
        config.CurrentLimits.StatorCurrentLimit = 40;
        intakeRollerMotor.getConfigurator().apply(config);
        intakeRollerMotor.set(0);
        m_VelocityVoltage.withSlot(0);
    }

    @Override
    public void periodic() {
        m_VelocityVoltage.withVelocity(target);
        //intakeRollerMotor.setControl(m_VelocityVoltage);
        SmartDashboard.putNumber("Intake Roller Speed", m_VelocityVoltage.Velocity);
    }

    public void stop() {
        target = 0;
    }

    public void setTarget(double targetSpeed){
        target = targetSpeed;
    }

    public void incrementRollers(double rotationChange) {
        this.ampTarget += rotationChange;
    
    }

    public Boolean hasGamepiece() {
        if(intakeRollerMotor.getSupplyCurrent(true).getValueAsDouble() > 10){
            return true;
        }
        return false;
    }
    
    public Command stopC(){
        Command result = runOnce(this::stop);
        return result;
    }

    public Command setTargetC(double targetSpeed){
        Command result = runOnce(() -> setTarget(targetSpeed));
        return result;
    }

    public Command incrementRollersC(double rotationChange){
        Command result = runOnce(()-> incrementRollers(rotationChange));
        return result;
    } 

    /*
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe rotational distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Angle> m_rotation = mutable(Rotations.of(0));
    // Mutable holder for unit-safe rotational  velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
    private final VoltageOut m_voltageOut = new VoltageOut(0);
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        // Using amperage, since it is converted when it comes out it ok.  Config expecst type volts.
        new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)), Volts.of(7), null, null),
        new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            (Measure<Voltage> volts) -> {
                intakeRollerMotor.setControl(m_voltageOut.withOutput(volts.in(Volts)));
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism
            // being
            // characterized.
            log -> {
                // Record a frame for the wheel motor.
                log.motor("wheel")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            intakeRollerMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_rotation.mut_replace(intakeRollerMotor.getPosition().getValueAsDouble(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(intakeRollerMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));

            },
            // Tell SysId to make generated commands require this subsystem, suffix test
            // state in
            // WPILog with this subsystem's name ("LauncherRollers")
            this));

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }*/
   
}
