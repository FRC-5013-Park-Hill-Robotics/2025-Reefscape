package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX ElevatorLeftMotor = new TalonFX(CANConstants.ELEVATOR_LEFT_ID, CANConstants.CANBUS_ELEVATOR);
    private final TalonFX ElevatorRightMotor = new TalonFX(CANConstants.ELEVATOR_RIGHT_ID, CANConstants.CANBUS_ELEVATOR);

    //private final DutyCycleEncoder encoderLeft = new DutyCycleEncoder(CANConstants.ELEVATOR_LEFT_ENCODER_DIO);
    //private final DutyCycleEncoder encoderRight = new DutyCycleEncoder(CANConstants.ELEVATOR_RIGHT_ENCODER_DIO, 10,0);
    
    private final Follower rightFollow = new Follower(CANConstants.ELEVATOR_LEFT_ID, true);

    private final PIDController eController = new PIDController(1.2, 0, 0);
    private final SlewRateLimiter limiter = new SlewRateLimiter(36);
    private final Debouncer stopDown = new Debouncer(0.1);

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

        double output = eController.calculate(getPosition(), setpoint);
        output = limiter.calculate(MathUtil.clamp(output, -ElevatorConstants.maxVoltage, ElevatorConstants.maxVoltage)) + ElevatorConstants.feedforward;
        SmartDashboard.putNumber("elevatorEncoderPIDOutput", output);

        SmartDashboard.putNumber("elevatorVolL", ElevatorLeftMotor.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevatorVolR", ElevatorRightMotor.getSupplyVoltage().getValueAsDouble());

        SmartDashboard.putNumber("elevatorVelocity", ElevatorRightMotor.getVelocity().getValueAsDouble());

        //Going down resets encoder
        // if(stopDown.calculate(output > 0 && Math.abs(ElevatorLeftMotor.getVelocity().getValueAsDouble()) < 0.2)){
        //     ElevatorLeftMotor.setPosition(0);
        //     ElevatorRightMotor.setPosition(0);
        //     setpoint = -0.1;  
        // }

        //Going too high stops motor
        if(output > 0 && getPosition() > ElevatorConstants.MaxHeight){
            //output = 0;
        }
        
        //Going too high while wrist up stops motor
        if(output > 0 && getPosition() > ElevatorConstants.MaxHeightWhenWristUp){
            //output = 0;
        }

        ElevatorLeftMotor.setVoltage(output);
    }

    public double getPosition(){
        return ElevatorLeftMotor.getPosition().getValueAsDouble();
    }

    public boolean atPos(){
        if(Math.abs(ElevatorLeftMotor.getPosition().getValueAsDouble()-setpoint)<1){
            return true;
        }
        else{
            return false;
        }
    }

    public void setPos(double newPos) {
        if(newPos < ElevatorConstants.UpperHardLimit){
            newPos = ElevatorConstants.UpperHardLimit;
        }
        if(newPos > ElevatorConstants.LowerHardLimit){
            newPos = ElevatorConstants.LowerHardLimit;
        }
        setpoint = newPos;
    }

    public void incrementPos(double add) {
        setpoint += add;
    }

    public void zero(){
        ElevatorLeftMotor.setPosition(0);
        ElevatorRightMotor.setPosition(0);
        setpoint = -0.1;  
    }

    public Command waitUntilAtPosC() {
        Command result = Commands.waitUntil(this::atPos);
        return result;
    }

    public Command setPosC(double newPos) {
        Command result = runOnce(() ->  setPos(newPos));
        return result;
    }

    public Command setPosUntilC(double newPos) {
        Command result = runOnce(() ->  setPos(newPos)).until(this::atPos);
        return result;
    }

    public Command incrementPosC(double add) {
        Command result = runOnce(() ->  incrementPos(add));
        return result;
    }

    public Command zeroC() {
        Command result = runOnce(() ->  zero());
        return result;
    }

}
