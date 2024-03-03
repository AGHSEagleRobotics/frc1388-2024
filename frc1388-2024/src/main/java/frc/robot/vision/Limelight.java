package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {

  private final DriveTrainSubsystem m_driveTrain;
  private static NetworkTable m_table;

  /** Creates a new Limelight. */
  public Limelight(DriveTrainSubsystem driveTrain) {
    m_table = NetworkTableInstance.getDefault().getTable("limelight-shooter");
    m_driveTrain = driveTrain;
    // double rz = (bot_pose_blue[5] + 360) % 360; (test what this does later)

  }

  public enum Advanced_Target {

    kone(0), ktwo(1), kthree(2);

    private static final Map<Integer, Advanced_Target> MY_MAP = new HashMap<Integer, Advanced_Target>();

    static {
      for (Advanced_Target Advanced_Target : values()) {
        MY_MAP.put(Advanced_Target.getValue(), Advanced_Target);
      }
    }

    private Integer value;

    private Advanced_Target(Integer value) {
      this.value = value;
    }

    public Integer getValue() {
      return value;
    }

    public static Advanced_Target getByValue(Integer value) {
      return MY_MAP.get(value);
    }

    public String toString() {
      return name();
    }

  }

  /**
   * tx Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
   * @param i 
   * 
   * @return
   */

   public double getAprilTagID() {
    NetworkTableEntry tid = m_table.getEntry("tid");
    double m_tid = tid.getDouble(0);
    return m_tid;
   }

  public boolean getIsTargetFound() {
    NetworkTableEntry tv = m_table.getEntry("tv");
    double m_tv = tv.getDouble(0);
    if (m_tv == 0.0f) {
      return false;
    } else {
      return true;
    }
  }

  /**
   * ty Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
   * can go higher than that
   * @return
   */
  public double getdegVerticalToTarget() {
    NetworkTableEntry ty = m_table.getEntry("ty");
    double m_ty = ty.getDouble(0.0);
    return m_ty;
  }

  public double getdegRotationToTarget() {
    NetworkTableEntry tx = m_table.getEntry("tx");
    double m_tx = tx.getDouble(0.0);
    return m_tx;
  }

  /**
   * ta Target Area (0% of image to 100% of image)
   * 
   * @return
   */
  public double getTargetArea() {
    NetworkTableEntry ta = m_table.getEntry("ta");
    double m_ta = ta.getDouble(0.0);
    return m_ta;
  }

  /**
   * ts Skew or rotation (-90 degrees to 0 degrees)
   * 
   * @return
   */
  public double getSkew_Rotation() {
    NetworkTableEntry ts = m_table.getEntry("ts");
    double m_ts = ts.getDouble(0.0);
    return m_ts;
  }

  /**
   * tl The pipeline’s latency contribution (ms) Add at least 11ms for image
   * capture latency.
   * 
   * @return
   */
  public double getPipelineLatency() {
    NetworkTableEntry tl = m_table.getEntry("tl");
    double m_tl = tl.getDouble(0.0);
    return m_tl;
  }

  private void resetPilelineLatency() {
    m_table.getEntry("tl").setValue(0.0);
  }

  public double getAdvanced_RotationToTarget(Advanced_Target raw) {
    NetworkTableEntry txRaw = m_table.getEntry("tx" + Integer.toString(raw.getValue()));
    double x = txRaw.getDouble(0.0);
    SmartDashboard.putNumber("Get Advanced Horizontal Degree", x);
    return x;
  }

  public double getAdvanced_degVerticalToTarget(Advanced_Target raw) {
    NetworkTableEntry tyRaw = m_table.getEntry("ty" + Integer.toString(raw.getValue()));
    double y = tyRaw.getDouble(0.0);
    SmartDashboard.putNumber("Get Advanced Vertical Degree", y);
    return y;
  }

  public double getAdvanced_TargetArea(Advanced_Target raw) {
    NetworkTableEntry taRaw = m_table.getEntry("ta" + Integer.toString(raw.getValue()));
    double a = taRaw.getDouble(0.0);
    SmartDashboard.putNumber("Get Advanced Target Area", a);
    return a;
  }

  public double getAdvanced_Skew_Rotation(Advanced_Target raw) {
    NetworkTableEntry tsRaw = m_table.getEntry("ts" + Integer.toString(raw.getValue()));
    double s = tsRaw.getDouble(0.0);
    SmartDashboard.putNumber("Advanced Skew Rotation Degrees", s);
    return s;

  }

  public double getDistance() {
    double[] targetSpace = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
        .getDoubleArray(new double[] {});

    double[] camerapose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace")
        .getDoubleArray(new double[] {});

    double[] robotspace = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace")
        .getDoubleArray(new double[] {});
    // finds distance in meters (needs callibration from limelight) needs to see the
    // april tag (needs testing)

    if (getIsTargetFound() == true) {

      double distance;
      distance = Math
          .sqrt((targetSpace[0] * targetSpace[0]) + (targetSpace[1] * targetSpace[1])
              + (targetSpace[2] * targetSpace[2]));

      // SmartDashboard.putNumber("Target Space 1", targetSpace[1]);
      // SmartDashboard.putNumber("Target Space 2", targetSpace[2]);
      // SmartDashboard.putNumber("Target Space 3", targetSpace[3]);
      // SmartDashboard.putNumber("Target Space 4", targetSpace[4]);
      // SmartDashboard.putNumber("Target Space 5", targetSpace[5]);
      // SmartDashboard.putNumber("Target Space 6", targetSpace[6]);

      // SmartDashboard.putNumber("camera Space 1", camerapose[1]);
      // SmartDashboard.putNumber("camera Space 2", camerapose[2]);
      // SmartDashboard.putNumber("camera Space 3", camerapose[3]);s
      // SmartDashboard.putNumber("camera Space 4", camerapose[4]);
      // SmartDashboard.putNumber("camera Space 5", camerapose[5]);
      // SmartDashboard.putNumber("camera Space 6", camerapose[6]);

      // SmartDashboard.putNumber("Robot Space 1", robotspace[1]);
      // SmartDashboard.putNumber("Robot Space 2", robotspace[2]);
      // SmartDashboard.putNumber("Robot Space 3", robotspace[3]);
      // SmartDashboard.putNumber("Robot Space 4", robotspace[4]);
      // SmartDashboard.putNumber("Robot Space 5", robotspace[5]);
      // SmartDashboard.putNumber("Robot Space 6", robotspace[6]);
      return distance;
    }

    return 0;
  }

  public void turnToSpeaker() {

      if (getdegRotationToTarget() > 0) {
        if (getAngleFromSpeaker() > 4) {
          m_driveTrain.drive(0, 0, 0.5);
        }
      }
      if (getdegRotationToTarget() < 0) {
        if (getAngleFromSpeaker() < -4) {
          m_driveTrain.drive(0, 0, -0.5);
        }
      } else if (getdegRotationToTarget() >= 4 && getdegRotationToTarget() <= -4) {
        m_driveTrain.drive(0, 0, 0);

    }
  }

  public void goToSpeaker() {

      if (getDistance() > 3) {
        
      } else if (getDistance() < 3.5) {
        
      } else if (getDistance() >= 3 && getDistance() <= 3.5) {
        m_driveTrain.drive(0, 0, 0);
      }
  }

  public void goToCenterOfSpeaker() {
    double[] targetSpace = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
        .getDoubleArray(new double[] {});

        if(targetSpace[0] >= 0.5)
        {
          m_driveTrain.drive(0, -0.5, 0);
        }
  
        if(targetSpace[0] <= -0.5)
        {
          m_driveTrain.drive(0, 0.5, 0);
        }
    }
  

  public double getAngleFromSpeaker() {
    return getdegRotationToTarget();
  }

  public void setLimelightLEDsOn(Boolean on) {
    if (on) {
      m_table.getEntry("ledMode").setNumber(2);
    } else {
      m_table.getEntry("ledMode").setNumber(1);
    }
  }

  @Override
  public void periodic() {
    if (getIsTargetFound() == true) {
      SmartDashboard.putNumber("Distance to April Tag: ", getDistance()); // it calculates in meters
      SmartDashboard.putBoolean("Is the target found", getIsTargetFound());
      SmartDashboard.putNumber("Get Horizontal Degree", getdegRotationToTarget());
      SmartDashboard.putNumber("Get Vertical Degree", getdegVerticalToTarget());
      SmartDashboard.putNumber("Get Skew Degree", getSkew_Rotation());

      SmartDashboard.putNumber("Get Vertical Degree", getdegRotationToTarget());

      SmartDashboard.putNumber("April Tag IDS", getAprilTagID());
    }
  }
}