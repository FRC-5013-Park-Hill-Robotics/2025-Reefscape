package frc.robot.constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class FieldPositions {
    public static final double fieldLengthX = Units.inchesToMeters(690.876);
    public static final double fieldWidthY = Units.inchesToMeters(317);

    public static Pose2d BlueToRedPose(Pose2d input){
        return new Pose2d(fieldLengthX-input.getX(), fieldWidthY-input.getY(), input.getRotation()/*.minus(Rotation2d.fromDegrees(180))*/);
    }

    public static Pose2d CenterPose(Pose2d pos1, Pose2d pos2){
        return new Pose2d((pos1.getX()+pos2.getX())/2, (pos1.getY()+pos2.getY())/2, pos1.getRotation());
    }

    public static final Pose2d BlueA = new Pose2d(3.29, 4.29, Rotation2d.fromDegrees(0));
    public static final Pose2d BlueB = new Pose2d(3.29, 3.99, Rotation2d.fromDegrees(0));
    public static final Pose2d BlueC = new Pose2d(3.73, 3.33, Rotation2d.fromDegrees(60));
    public static final Pose2d BlueD = new Pose2d(3.93, 3.12, Rotation2d.fromDegrees(60));
    public static final Pose2d BlueE = new Pose2d(5.04, 2.89, Rotation2d.fromDegrees(120));
    public static final Pose2d BlueF = new Pose2d(5.35, 3.14, Rotation2d.fromDegrees(120));
    public static final Pose2d BlueG = new Pose2d(5.77, 3.99, Rotation2d.fromDegrees(-180));
    public static final Pose2d BlueH = new Pose2d(5.76, 4.29, Rotation2d.fromDegrees(-180));
    public static final Pose2d BlueI = new Pose2d(5.37, 5.03, Rotation2d.fromDegrees(-120));
    public static final Pose2d BlueJ = new Pose2d(5.03, 5.23, Rotation2d.fromDegrees(-120));
    public static final Pose2d BlueK = new Pose2d(4.12, 5.27, Rotation2d.fromDegrees(-60));
    public static final Pose2d BlueL = new Pose2d(3.78, 5.05, Rotation2d.fromDegrees(-60));

    public static final Pose2d RedA = BlueToRedPose(BlueA);
    public static final Pose2d RedB = BlueToRedPose(BlueB);
    public static final Pose2d RedC = BlueToRedPose(BlueC);
    public static final Pose2d RedD = BlueToRedPose(BlueD);
    public static final Pose2d RedE = BlueToRedPose(BlueE);
    public static final Pose2d RedF = BlueToRedPose(BlueF);
    public static final Pose2d RedG = BlueToRedPose(BlueG);
    public static final Pose2d RedH = BlueToRedPose(BlueH);
    public static final Pose2d RedI = BlueToRedPose(BlueI);
    public static final Pose2d RedJ = BlueToRedPose(BlueJ);
    public static final Pose2d RedK = BlueToRedPose(BlueK);
    public static final Pose2d RedL = BlueToRedPose(BlueL);

    public static final Pose2d BlueC1 = CenterPose(BlueA, BlueB);
    public static final Pose2d BlueC2 = CenterPose(BlueC, BlueD);
    public static final Pose2d BlueC3 = CenterPose(BlueE, BlueF);
    public static final Pose2d BlueC4 = CenterPose(BlueG, BlueH);
    public static final Pose2d BlueC5 = CenterPose(BlueI, BlueJ);
    public static final Pose2d BlueC6 = CenterPose(BlueK, BlueL);

    public static final Pose2d RedC1 = BlueToRedPose(BlueC1);
    public static final Pose2d RedC2 = BlueToRedPose(BlueC2);
    public static final Pose2d RedC3 = BlueToRedPose(BlueC3);
    public static final Pose2d RedC4 = BlueToRedPose(BlueC4);
    public static final Pose2d RedC5 = BlueToRedPose(BlueC5);
    public static final Pose2d RedC6 = BlueToRedPose(BlueC6);
    
    public static final List<Pose2d> Left = new ArrayList<Pose2d>(Arrays.asList(RedA, RedC, RedE, RedG,RedI,RedK,
                                                                                BlueA, BlueC, BlueE, BlueG, BlueI, BlueK));
    public static final List<Pose2d> Right = new ArrayList<Pose2d>(Arrays.asList(RedB, RedD, RedF, RedH, RedJ, RedL,
                                                                                BlueB, BlueD, BlueF, BlueH, BlueJ, BlueL));
    public static final List<Pose2d> Center = new ArrayList<Pose2d>(Arrays.asList(BlueC1, BlueC2, BlueC3, BlueC4, BlueC5, BlueC6, 
                                                                                RedC1, RedC2, RedC3, RedC4, RedC5, RedC6));
}
