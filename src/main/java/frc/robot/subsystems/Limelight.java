package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.trobot5013lib.LimelightHelpers;
//import webblib.util.RectanglePoseArea;

public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */
  //public static final RectanglePoseArea field =
  //new RectanglePoseArea(new Translation2d(0.0, 0.0), new Translation2d(16.54, 8.02));
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv; 

  private boolean aprilTagViable;
  private RobotContainer m_robotContainer;
  private Alliance alliance;
  private Boolean enable = true;
  private Boolean trust = false;
  private int fieldError = 0;
  private int distanceError = 0;
  private Pose2d botpose;
  private String name;
  private Boolean doRejectUpdate;

  private CommandSwerveDrivetrain mDrivetrain;

  public LimeLight(String name, boolean aprilTagViable) {
    /**
     * tx - Horizontal Offset
     * ty - Vertical Offset 
     * ta - Area of target 
     * tv - Target Visible
     */
    this.name = name;
    this.aprilTagViable = aprilTagViable;

    this.table = NetworkTableInstance.getDefault().getTable(name);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    doRejectUpdate = false;

    mDrivetrain = RobotContainer.getInstance().getDrivetrain();

    //LimelightHelpers.SetRobotOrientation(name, mDrivetrain.getState().RawHeading.getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(name, mDrivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    Double targetDistance = LimelightHelpers.getTargetPose3d_CameraSpace(name).getTranslation().getDistance(new Translation3d());
    Double confidence = 1 - ((targetDistance - 1) / 6);
    // if(alliance == Alliance.Red){
    //   mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    // }
    // if(alliance == Alliance.Blue){
    //   mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    // }

    // SmartDashboard.putNumber("LL_Confidence"+name, confidence);
    // SmartDashboard.putBoolean("LL_diffBoolean"+name, mDrivetrain.getState().Pose.getTranslation().getDistance(mt2.pose.getTranslation()) < 0.5);
    //SmartDashboard.putNumber("MT2 Drivetrain", mDrivetrain.getState().Pose.getRotation().getDegrees());
    //SmartDashboard.putNumber("MT2 Heading", mt2.pose.getRotation().getDegrees());

    //boolean PoseEstimating = false;
    if(mt2 != null && aprilTagViable){
      // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      if(Math.abs(Math.toDegrees(mDrivetrain.getState().Speeds.omegaRadiansPerSecond)) > 720) {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if((!doRejectUpdate)&&true)//(trust == true || mDrivetrain.getState().Pose.getTranslation().getDistance(mt1.pose.getTranslation()) < 0.5))
      {
        mDrivetrain.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds,
            VecBuilder.fill(confidence, confidence, 9999999) //Maybe try 9999 for n3? Was 0.01
        );
      }
      // SmartDashboard.putBoolean("LL_Boolean"+name, (!doRejectUpdate) && mDrivetrain.getState().Pose.getTranslation().getDistance(mt2.pose.getTranslation()) < 0.5);
      // SmartDashboard.putBoolean("LL_PoseEstimating"+name, PoseEstimating);
    }
  }

  public void setAprilTagViable(Boolean state){
    aprilTagViable = state;
  }

  public void setTrust(boolean newTrust){
    trust = newTrust;
  }
  public void setAlliance(Alliance alliance) {
    this.alliance = alliance;
  }

  public NetworkTableEntry getTx() {
    return tx;
  }

  public NetworkTableEntry getTy() {
    return ty;
  }

  public NetworkTableEntry getTa() {
    return ta;
  }

  public double getTxAngleRadians() {
    return Units.degreesToRadians(tx.getDouble(0));
  }

  public double getTargetAngleRadians() {
    //return getTxAngleRadians()+m_robotContainer.getDrivetrain().getPose().getRotation().getRadians();
    return getTxAngleRadians()+Math.toRadians(m_robotContainer.getDrivetrain().getRotation3d().getAngle());
  }
  public boolean hasTarget(){
    SmartDashboard.putNumber("tv; ", tv.getDouble(0));
    return tv.getDouble(0) != 0;
  }
  public void setPipeline(int pipeline){
    table.getEntry("pipeline").setNumber(pipeline);
  }

  public Command setPipelineC(int pipeline){
    Command result = runOnce(() -> setPipeline(pipeline));
    return result;
  }

  public Command setTrustC(boolean trust){
    Command result = runOnce(() -> setTrust(trust));
    return result;
  }

  public Command setAprilTagViableC(Boolean state){
    Command result = runOnce(() -> setAprilTagViable(state));
    return result;
  }
}


