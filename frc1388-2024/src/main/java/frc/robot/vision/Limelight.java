package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight extends SubsystemBase {

  private static NetworkTable m_shooterTable;
  private static NetworkTable m_intakeTable;

  /** Creates a new Limelight. */
  public Limelight(String shooterName, String intakeName) {
    m_shooterTable = NetworkTableInstance.getDefault().getTable(shooterName);
    m_intakeTable = NetworkTableInstance.getDefault().getTable(intakeName);
    
  }

  public void setShooterPipeline(double pipelineNumber) {
    NetworkTableEntry shooterPipeline = m_shooterTable.getEntry("pipeline");
    shooterPipeline.setNumber(pipelineNumber);
  }

  public void setIntakePipeline(double pipelineNumber) {
    NetworkTableEntry intakePipeline = m_intakeTable.getEntry("pipeline");
    intakePipeline.setNumber(pipelineNumber);
  }

   public double getAprilTagID() {
    NetworkTableEntry tid = m_shooterTable.getEntry("tid");
    double m_tid = tid.getDouble(0);
    return m_tid;
   }

  public boolean getApriltagTargetFound() {
    NetworkTableEntry tv = m_shooterTable.getEntry("tv");
    double m_tv = tv.getDouble(0);
    if (m_tv == 0.0f) {
      return false;
    } else {
      return true;
    }
  }

  public boolean getIsNoteFound() {
    NetworkTableEntry tv = m_intakeTable.getEntry("tv");
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
  public double getAprilTagTy() {
    NetworkTableEntry ty = m_shooterTable.getEntry("ty");
    double m_ty = ty.getDouble(0.0);
    return m_ty;
  }

  public double getNoteTy() {
    NetworkTableEntry ty = m_intakeTable.getEntry("ty");
    double m_ty = ty.getDouble(0.0);
    return m_ty;
  }

  public double getAprilTagTx() {
    NetworkTableEntry tx = m_shooterTable.getEntry("tx");
    double m_tx = tx.getDouble(0.0);
    m_tx += LimelightConstants.TX_OFFSET;
    return m_tx;
  }

  public double getNoteTx() {
    NetworkTableEntry tx = m_intakeTable.getEntry("tx");
    double m_tx = tx.getDouble(0.0);
    m_tx += LimelightConstants.TX_OFFSET;
    return m_tx;
  }

  /**
   * ta Target Area (0% of image to 100% of image)
   * 
   * @return
   */
  public double getAprilTagTargetArea() {
    NetworkTableEntry ta = m_shooterTable.getEntry("ta");
    double m_ta = ta.getDouble(0.0);
    return m_ta;
  }

  public double getNoteTargetArea() {
    NetworkTableEntry ta = m_intakeTable.getEntry("ta");
    double m_ta = ta.getDouble(0.0);
    return m_ta;
  }
  /**
   * ts Skew or rotation (-90 degrees to 0 degrees)
   * 
   * @return
   */
  public double getSkew_Rotation() {
    NetworkTableEntry ts = m_shooterTable.getEntry("ts");
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
    NetworkTableEntry tl = m_shooterTable.getEntry("tl");
    double m_tl = tl.getDouble(0.0);
    return m_tl;
  }

  private void resetPilelineLatency() {
    m_shooterTable.getEntry("tl").setValue(0.0);
  }

  public double getDistance() {
    // double[] targetSpace = m_shooterTable.getEntry("targetpose_cameraspace")
    //     .getDoubleArray(new double[] {});

    // // finds distance in meters (needs callibration from limelight) needs to see the
    // // april tag (needs testing)
    // // might also need to add values like where the limelight is although we might be able to do that in the pipeline

    // if (targetSpace != null) {
    //   double distance;
    //   distance = targetSpace[2];

    //   return distance;
    // }
    return 0;
  }

  public double getAngleFromSpeaker() {
    return getAprilTagTx();
  }

  public void setLimelightLEDsOn(Boolean on) {
    if (on) {
      m_shooterTable.getEntry("ledMode").setNumber(LimelightConstants.LED_FORCE_BLINK);
      m_intakeTable.getEntry("ledMode").setNumber(LimelightConstants.LED_FORCE_BLINK);
    } else {
      m_shooterTable.getEntry("ledMode").setNumber(LimelightConstants.LED_FORCE_OFF);
      m_intakeTable.getEntry("ledMode").setNumber(LimelightConstants.LED_FORCE_OFF);
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      setShooterPipeline(0);
    }
    else {
      setShooterPipeline(1);
    }
      SmartDashboard.putNumber("Distance to April Tag", getDistance()); // it calculates in meters
      SmartDashboard.putBoolean("Is the target found", getApriltagTargetFound());
      SmartDashboard.putNumber("Get April Tag Tx", getAprilTagTx());
      SmartDashboard.putNumber("Get Note Tx", getNoteTx());
      SmartDashboard.putNumber("Get April Tag Ty", getAprilTagTy());
      SmartDashboard.putNumber("Get Note Ty", getNoteTx());
      SmartDashboard.putNumber("Get Skew Degree", getSkew_Rotation());

      SmartDashboard.putNumber("Get Vertical Degree", getAprilTagTx());

      SmartDashboard.putNumber("April Tag IDS", getAprilTagID());
      

  }
}