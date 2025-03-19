package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.generated.TunerConstants;

public class DriveConstants {
    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final double movementLimitAmount = 50; //Given to x/y slew rate limiters
    public static final double limit = 0.5;

    public static final PIDController ControllerX = new PIDController(10, 0, 0);
    public static final PIDController ControllerY = new PIDController(10, 0, 0);
    public static final PIDController ControllerH = new PIDController(0.15, 0, 0);
}
