package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;

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
   
    // double rz = (bot_pose_blue[5] + 360) % 360; (test what this does later)

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
    double[] targetSpace = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
        .getDoubleArray(new double[] {});

    double[] camerapose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace")
        .getDoubleArray(new double[] {});

    double[] robotspace = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace")
        .getDoubleArray(new double[] {});
    // finds distance in meters (needs callibration from limelight) needs to see the
    // april tag (needs testing)
    if (targetSpace.length >= 3) {
      if (targetSpace[0] != 0 && targetSpace[1] != 0 && targetSpace[2] != 0) {
        double distance = Math.sqrt(targetSpace[0] * targetSpace[0] + targetSpace[1] * targetSpace[1] + targetSpace[2] * targetSpace[2]);
        SmartDashboard.putNumber("Distance to April Tag: ", distance); // it calculates in meters
        SmartDashboard.putNumber("Target Space 1", targetSpace[1]);
        SmartDashboard.putNumber("Target Space 2", targetSpace[2]);
        SmartDashboard.putNumber("Target Space 3", targetSpace[3]);
        SmartDashboard.putNumber("Target Space 4", targetSpace[4]);
        SmartDashboard.putNumber("Target Space 5", targetSpace[5]);
        SmartDashboard.putNumber("Target Space 6", targetSpace[6]);

        SmartDashboard.putNumber("camera Space 1", camerapose[1]);
        SmartDashboard.putNumber("camera Space 2", camerapose[2]);
        SmartDashboard.putNumber("camera Space 3", camerapose[3]);
        SmartDashboard.putNumber("camera Space 4", camerapose[4]);
        SmartDashboard.putNumber("camera Space 5", camerapose[5]);
        SmartDashboard.putNumber("camera Space 6", camerapose[6]);

        SmartDashboard.putNumber("Robot Space 1", robotspace[1]);
        SmartDashboard.putNumber("Robot Space 2", robotspace[2]);
        SmartDashboard.putNumber("Robot Space 3", robotspace[3]);
        SmartDashboard.putNumber("Robot Space 4", robotspace[4]);
        SmartDashboard.putNumber("Robot Space 5", robotspace[5]);
        SmartDashboard.putNumber("Robot Space 6", robotspace[6]);
        return distance;
      }
    }
    return 0;
  }

  public static double getAngleFromSpeaker() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    return tx; // might be angle need to test
  }



  @Override
  public void periodic() {    
    SmartDashboard.putNumber("Tx: " , getAngleFromSpeaker());
    SmartDashboard.putNumber("Distance From AprilTag:", calculateDistance());
    
  }
}