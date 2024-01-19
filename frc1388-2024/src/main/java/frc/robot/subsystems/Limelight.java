package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {

  private final NetworkTableEntry m_tx;
  private double txPrev;
  private final NetworkTableEntry m_ty;
  private final NetworkTableEntry m_tz;
  private final NetworkTableEntry m_ta;
  private final NetworkTableEntry m_tv;
  private final NetworkTableEntry m_ts;
  private final NetworkTableEntry m_tp;
  private final NetworkTableEntry m_tshort;
  private final NetworkTableEntry m_tlong;
  private final NetworkTableEntry m_thor;
  private final NetworkTableEntry m_tvert;
  private final NetworkTableEntry m_botpos;
  private final double m_xOffset;
  private final double m_yOffset;
  private final double m_zOffset;
  private int tick = 0;

  private final NetworkTable m_table;

  /** Creates a new Limelight. */
  public Limelight() {

    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    m_tx = m_table.getEntry("tx");
    m_ty = m_table.getEntry("ty");
    m_tz = m_table.getEntry("tz");
    m_ta = m_table.getEntry("ta");
    m_tv = m_table.getEntry("tv");
    m_ts = m_table.getEntry("ts");
    m_tp = m_table.getEntry("tp");
    m_tshort = m_table.getEntry("tshort");
    m_tlong = m_table.getEntry("tlong");
    m_thor = m_table.getEntry("thor");
    m_tvert = m_table.getEntry("tvert");
    m_botpos = m_table.getEntry("botpose");
    m_xOffset = m_table.getEntry("camtran").getDoubleArray(new double[]{0.0 , 0.0})[0];
    m_yOffset = m_table.getEntry("camtran").getDoubleArray(new double[]{0.0 , 0.0})[3];
    m_zOffset = m_table.getEntry("camtran").getDoubleArray(new double[]{0.0 , 0.0})[2];
   


    // double rz = (bot_pose_blue[5] + 360) % 360; (test what this does later)

  }
	
  public double getXOffset() {
    return m_xOffset;
  }

  public double getYOffset() {
    return m_yOffset;
  }

  public double getZOffset() {
    return m_zOffset;
  }

  public double getTx() {
    return m_tx.getDouble(0.0);
  }

  public double getTy() {
    return m_ty.getDouble(0.0);
  }

  public double getTz() {
    return m_tz.getDouble(0.0);
  }

  public double getTa() {
    return m_ta.getDouble(0.0);
  }

  public double getTv() {
    return m_tv.getDouble(0.0);
  }


  public static double calculateDistance() {
    double[] camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
        .getDoubleArray(new double[] {});
    // finds distance in meters (needs callibration from limelight) needs to see the
    // april tag (needs testing)
    if (camtran.length >= 3) {
      if (camtran[0] != 0 && camtran[1] != 0 && camtran[2] != 0) {
        double distance = Math.sqrt(camtran[0] * camtran[0] + camtran[1] * camtran[1] + camtran[2] * camtran[2]);
        SmartDashboard.putNumber("Distance to April Tag: ", distance); // it calculates in meters
        return distance;
      }
    }
    return 0;
  }
  @Override
  public void periodic() {

    // double[] bot_pose_blue = m_table
    // .getEntry("botpose_wpiblue")
    // .getDoubleArray(new double[1]);

    // double tx = bot_pose_blue[0];
    // double ty = bot_pose_blue[1];
    // double tz = bot_pose_blue[2];
    // double rx = bot_pose_blue[3];
    // double ry = bot_pose_blue[4];
    
    
    
    // SmartDashboard.putNumber("Tx", tx);  //tx x axis left right 
    // SmartDashboard.putNumber("Ty", ty);  //ty y axis up and down
    // SmartDashboard.putNumber("Tz", tz);  //tz forwards and backwards in meters
    // SmartDashboard.putNumber("Rx", rx);
    // SmartDashboard.putNumber("Ry", ry);
    // // This method will be called once per scheduler run
    // PowerDistribution pdh = new PowerDistribution();
    // System.out.println("pdh 0: " + pdh.getCurrent(0) + " pdh 2: " +
    // pdh.getCurrent(2));
    // System.out.println("ts: " + m_ts.getDouble(0.0));
    // System.out.println(m_tp.getDouble(0.0));
    SmartDashboard.putNumber("Distance From AprilTag:", calculateDistance());
    SmartDashboard.putNumber("target area", getTv());
    SmartDashboard.putNumber("target x", getTx());
    SmartDashboard.putNumber("target y", getTy());
    SmartDashboard.putNumber("target z", getTz());
    SmartDashboard.putNumber("target short", m_tshort.getDouble(0.0));
    SmartDashboard.putNumber("target long", m_tlong.getDouble(0.0));
    SmartDashboard.putNumber("target hor", m_thor.getDouble(0.0));
    SmartDashboard.putNumber("target vert", m_tvert.getDouble(0.0));
    SmartDashboard.putNumber("camtran[0]", getXOffset());
    SmartDashboard.putNumber("camtran[1]", getYOffset());
    SmartDashboard.putNumber("camtran[2]", getZOffset());
    double[] camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[]{});
    SmartDashboard.putNumber("camtran[0]0", camtran[0]);
    SmartDashboard.putNumber("camtran[1]1", camtran[1]);
    SmartDashboard.putNumber("camtran[2]2", camtran[2]);
    

  }
}