package frc.robot.constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldPositions {
    public static final Pose2d RedA = new Pose2d(14.1, 3.86, Rotation2d.fromDegrees(180));
    public static final Pose2d RedB = new Pose2d(14.1, 4.2, Rotation2d.fromDegrees(180));
    public static final Pose2d RedC = new Pose2d(13.8, 5, Rotation2d.fromDegrees(-120));
    public static final Pose2d RedD = new Pose2d(13.5, 5.16, Rotation2d.fromDegrees(-120));
    //public static final Pose2d RedE = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    //public static final Pose2d RedF = new Pose2d(13.8, 3.8, Rotation2d.fromDegrees(180));
    //public static final Pose2d RedG = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    //public static final Pose2d RedH = new Pose2d(13.8, 3.8, Rotation2d.fromDegrees(180));
    //public static final Pose2d RedI = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    //public static final Pose2d RedJ = new Pose2d(13.8, 3.8, Rotation2d.fromDegrees(180));
    // public static final Pose2d RedK = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    // public static final Pose2d RedL = new Pose2d(13.4, 3.8, Rotation2d.fromDegrees(120));

    // public static final Pose2d BlueA = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    // public static final Pose2d BlueB = new Pose2d(13.8, 3.8, Rotation2d.fromDegrees(180));
    // public static final Pose2d BlueC = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    // public static final Pose2d BlueD = new Pose2d(13.8, 3.8, Rotation2d.fromDegrees(180));
    // //public static final Pose2d BlueE = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    // //public static final Pose2d BlueF = new Pose2d(13.8, 3.8, Rotation2d.fromDegrees(180));
    // //public static final Pose2d BlueG = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    // //public static final Pose2d BlueH = new Pose2d(13.8, 3.8, Rotation2d.fromDegrees(180));
    // //public static final Pose2d BlueI = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    // //public static final Pose2d BlueJ = new Pose2d(13.8, 3.8, Rotation2d.fromDegrees(180));
    // public static final Pose2d BlueK = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    // public static final Pose2d BlueL = new Pose2d(13.8, 3.8, Rotation2d.fromDegrees(180));

    public static final Pose2d RedCSL = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    public static final Pose2d RedCSR = new Pose2d(14.2, 3.8, Rotation2d.fromDegrees(180));
    
    public static final List<Pose2d> Left = new ArrayList<Pose2d>(Arrays.asList(RedA, RedC));
    public static final List<Pose2d> Right = new ArrayList<Pose2d>(Arrays.asList(RedB, RedD));
}
