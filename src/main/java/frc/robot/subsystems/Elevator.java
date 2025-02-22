package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.trobot5013lib.RevThroughBoreEncoder;

public class Elevator extends SubsystemBase {
    private final TalonFX ElevatorLeftMotor = new TalonFX(CANConstants.ELEVATOR_LEFT_ID);
    private final TalonFX ElevatorRightMotor = new TalonFX(CANConstants.ELEVATOR_RIGHT_ID);

    private final RevThroughBoreEncoder encoderLeft = new RevThroughBoreEncoder(CANConstants.ELEVATOR_LEFT_ENCODER_DIO);
    private final RevThroughBoreEncoder encoderRight = new RevThroughBoreEncoder(CANConstants.ELEVATOR_RIGHT_ENCODER_DIO);
    
    private final PIDController eController = new PIDController(10, 0, 0);

    private double setpoint = 0;

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoderLeft", encoderLeft.getAngleRaw());
        SmartDashboard.putNumber("encoderRight", encoderRight.getAngleRaw());

        double angleAvg = (encoderLeft.getAngleRaw()+encoderRight.getAngleRaw())/2;
        SmartDashboard.putNumber("encoderAvg", angleAvg);

        double output = eController.calculate(angleAvg, setpoint);
        SmartDashboard.putNumber("encoderPIDOutput", output);

        //ElevatorLeftMotor.setVoltage(output);
        //ElevatorRightMotor.setVoltage(output);
    }

    public void setPos(double newPos) {
        setpoint = newPos;
    }
}
