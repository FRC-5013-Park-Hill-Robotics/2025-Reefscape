package frc.robot.constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;


public class FieldPositions {
    
    public static Pose2d BlueToRedPose(Pose2d input){
        return new Pose2d(17.5-input.getX(), input.getY(), input.getRotation());
    }

    public static final Pose2d BlueA = new Pose2d(3.402, 4.09, Rotation2d.fromDegrees(-3.50));
    public static final Pose2d BlueB = new Pose2d(3.397, 3.714, Rotation2d.fromDegrees(-3.26));
    public static final Pose2d BlueC = new Pose2d(3.945, 2.879, Rotation2d.fromDegrees(54.89));
    public static final Pose2d BlueD = new Pose2d(4.099, 2.745, Rotation2d.fromDegrees(51.90));
    //public static final Pose2d BlueE = new Pose2d(4.122, 2.807, Rotation2d.fromDegrees(57.463));
    public static final Pose2d BlueE = new Pose2d(4.977, 3.088, Rotation2d.fromDegrees(120.201));
    public static final Pose2d BlueF = new Pose2d(5.082, 3.124, Rotation2d.fromDegrees(123.48));
    public static final Pose2d BlueG = new Pose2d(5.273, 4.032, Rotation2d.fromDegrees(-177.76));
    public static final Pose2d BlueH = new Pose2d(5.26, 4.45, Rotation2d.fromDegrees(-177.46));
    //public static final Pose2d BlueI = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    //public static final Pose2d BlueJ = new Pose2d(13.8, 3.8, Rotation2d.fromDegrees(180));
    public static final Pose2d BlueK = new Pose2d(3.204, 5.041, Rotation2d.fromDegrees(-57.092));
    public static final Pose2d BlueL = new Pose2d(2.868, 4.929, Rotation2d.fromDegrees(-57.229));
    
    public static final Pose2d RedA = BlueToRedPose(BlueA);
    public static final Pose2d RedB = BlueToRedPose(BlueB);
    public static final Pose2d RedC = BlueToRedPose(BlueC);
    public static final Pose2d RedD = BlueToRedPose(BlueD);
    public static final Pose2d RedE = BlueToRedPose(BlueE);
    public static final Pose2d RedF = BlueToRedPose(BlueF);
    public static final Pose2d RedG = BlueToRedPose(BlueG);
    public static final Pose2d RedH = BlueToRedPose(BlueH);
    //public static final Pose2d RedI = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    //public static final Pose2d RedJ = new Pose2d(13.8, 3.8, Rotation2d.fromDegrees(180));
    public static final Pose2d RedK = BlueToRedPose(BlueK);
    public static final Pose2d RedL = BlueToRedPose(BlueL);

    public static final Pose2d RedCSL = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    public static final Pose2d RedCSR = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    
    public static final List<Pose2d> Left = new ArrayList<Pose2d>(Arrays.asList(RedA, RedC, RedE, RedG,/*RedI,*/RedK,
                                                                                BlueA, BlueC, BlueE, BlueG/*, BlueI*/, BlueK));
    public static final List<Pose2d> Right = new ArrayList<Pose2d>(Arrays.asList(RedB, RedD, RedF, RedH,/* RedJ,*/ RedL,
                                                                                BlueB, BlueD, BlueF, BlueH,/* BlueJ,*/ BlueL));
}
