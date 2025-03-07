package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.trobot5013lib.RevThroughBoreEncoder;

import edu.wpi.first.math.filter.Debouncer;

public class Elevator extends SubsystemBase {
    private final TalonFX ElevatorLeftMotor = new TalonFX(CANConstants.ELEVATOR_LEFT_ID, CANConstants.CANBUS_ELEVATOR);
    private final TalonFX ElevatorRightMotor = new TalonFX(CANConstants.ELEVATOR_RIGHT_ID, CANConstants.CANBUS_ELEVATOR);

    private final DutyCycleEncoder encoderLeft = new DutyCycleEncoder(CANConstants.ELEVATOR_LEFT_ENCODER_DIO);
    private final DutyCycleEncoder encoderRight = new DutyCycleEncoder(CANConstants.ELEVATOR_RIGHT_ENCODER_DIO, 10,0);
    
    private final Follower rightFollow = new Follower(CANConstants.ELEVATOR_LEFT_ID, true);

    private final PIDController eController = new PIDController(1, 0, 0);
    private final SlewRateLimiter limiter = new SlewRateLimiter(36);
    private final Debouncer stopDown = new Debouncer(0.2);

    private double setpoint = 0;

    public Elevator(){
        super();

        TalonFXConfiguration config1 = new TalonFXConfiguration();
        config1.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        ElevatorLeftMotor.getConfigurator().apply(config1);

        TalonFXConfiguration config2 = new TalonFXConfiguration();
        config2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        ElevatorRightMotor.getConfigurator().apply(config2);

        ElevatorRightMotor.setControl(rightFollow);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoderLeftK", ElevatorLeftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("encoderRightK", ElevatorRightMotor.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("elevatorPos", setpoint);

        double output = eController.calculate(ElevatorLeftMotor.getPosition().getValueAsDouble(), setpoint);
        output = limiter.calculate(MathUtil.clamp(output, -12.0, 12.0));
        SmartDashboard.putNumber("encoderPIDOutput", output);

        SmartDashboard.putNumber("elevatorVolL", ElevatorLeftMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevatorVolR", ElevatorRightMotor.getSupplyVoltage().getValueAsDouble());

        //Going down resets encoder
        if(stopDown.calculate(ElevatorLeftMotor.getSupplyVoltage().getValueAsDouble() < 0 /*&& ElevatorLeftMotor.*/)){
            //ElevatorLeftMotor.setPosition(0);
            //ElevatorRightMotor.setPosition(0);
        }

        //Going too high stops motor
        if(output > 0 && ElevatorLeftMotor.getPosition().getValueAsDouble() > 120){
            //output = 0;
        }

        ElevatorLeftMotor.setVoltage(output);
    }

    public void setPos(double newPos) {
        if(setpoint > 120){
            setpoint = 120;
        }
        //Allow for zeroing
        if(setpoint < -10){
            setpoint = 0;
        }
        setpoint = newPos;
    }

    public void incrementPos(double add) {
        setpoint += add;
    }

    public Command setPosC(double newPos) {
        Command result = runOnce(() ->  setPos(newPos));
        return result;
    }

    public Command incrementPosC(double add) {
        Command result = runOnce(() ->  setPos(add));
        return result;
    }

}
