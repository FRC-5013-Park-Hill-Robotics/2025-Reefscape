// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

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
        public static final double kP = 0.095013;
        public static final double kI = 0.0;
        public static final double kD = 0.00;
        public static final double kF = 0;
        public static final double kS = 0.5005013;
        public static final double kG = 0.672;
        public static final double kV = 0.12212;
        public static final double kA = 0.037683;
        public static final double kMaxSpeed =  2*Math.PI; //theoretical free speed of intake is 133.3333 radians per second we want to limit to 2pi
        public static final double kMaxAcceleration = kMaxSpeed * 3;
    }
    public final static class RollerGains {
        public static final double kP = 0.010371;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0;
        public static final double kS = 0.39545;
        public static final double kV = 0.12212;
        public static final double kA = 0.0046099;
        //public static final double kIntakeRotation = 36.2942;
       
    }

    public static final double MEUpLowerLimit = 0; //Motor Encoder range for Up (so we don't hit the top of elevator)
    public static final double MEUpUpperLimit = 0; //Motor Encoder range for Up (so we don't hit the top of elevator)
    public static final double feedforwardMod = -0.38;
    public static final double maxVoltage = 3;

    public static final double HasCoralBar = 2.8;
    public static final double hasAlgaeBar = 10;
    public static final double IntakeCoralSpeed = 60;
    public static final double IntakeAlgaeSpeed = 60;
    public static final double HoldAlgaeSpeed = 0.1;
    public static final double OutakeSpeed = 120;

    //When the wrist could potentially slam into the to of the elevator
    public static final double DangerZoneEUpper = -100;
    public static final double DangerZoneELower = -80;
    public static final double DangerZoneWRange = 40;
    public static final double DangerZoneWSetpoint = 40;

}
