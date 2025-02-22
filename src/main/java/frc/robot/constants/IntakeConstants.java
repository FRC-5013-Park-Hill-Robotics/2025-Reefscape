// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.constants.CANConstants;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public final class IntakeConstants {
    public final static int INTAKE_WRIST_MOTOR_CAN_ID = CANConstants.INTAKE_WRIST_ID;
    public final static int ENCODER_DIO_PORT = 1;
    public static final int INTAKE_ROLLER_ID = CANConstants.INTAKE_ROLLER_ID;
    public static final class RotationGains {
        public static final Rotation2d kPositionTolerance= Rotation2d.fromDegrees(2.5);
        //public static final double kP = 3.5;
        //public static final double kP = 3.75013;
        public static final double kP = 7.5013;
        public static final double kI = 0.0;
        public static final double kD = 0.4;
        public static final double kF = 0;
        public static final double kS = 0.5005013;
        public static final double kG = 0.672;
        public static final double kV = 0.12212;
        public static final double kA = 0.037683;
        public static final double kMaxSpeed =  2*Math.PI; //theoretical free speed of intake is 133.3333 radians per second we want to limit to 2pi
        public static final double kMaxAcceleration = kMaxSpeed * 3;
    }
    public final static class RollerGains {
        public static final double kP = 0.025371;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0.39545;
        public static final double kV = 0.12212;
        public static final double kA = 0.0046099;
        //public static final double kIntakeRotation = 36.2942;
       
    }

    public static final double kIntakeRotation = 50;
    public static final double kOuttakeRotation = -50;
    public static final double kAmpOut = -15;
    public static final double AMP_ANGLE_GROUND  =  Math.toRadians(78);// Math.toRadians(9.38);
    //old is 76
    public final static double DEPLOY_SETPOINT_TO_GROUND = Math.toRadians(-18);
    public final static double RETRACT_SETPOINT = 0;
    public static final int TIME_OF_FLIGHT_CAN_ID = 1;
    public static final double TIME_OF_FLIGHT_RANGE_MM = 110;
    public static final double ENCODER_OFFSET_RADIANS = Math.toRadians(6.7); //-114.5

}
