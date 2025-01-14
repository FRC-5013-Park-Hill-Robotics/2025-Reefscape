package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.subsystems.IntakeWrist.GroundRelativeGoal;

public class Elevator extends SubsystemBase {
    private final TalonFX ElevatorLeftMotor = new TalonFX(CANConstants.ELEVATOR_LEFT_ID);
    private final TalonFX ElevatorRightMotor = new TalonFX(CANConstants.ELEVATOR_RIGHT_ID);

    @Override
    public void periodic() {
        
    }
}
