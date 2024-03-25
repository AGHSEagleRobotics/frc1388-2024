package frc.robot.vision;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.Module.SetupContext;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private DriverStation.Alliance m_allianceColor;
  private static double[] botPose0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /** Creates a new Limelight. */
  public Limelight(String shooterName, String intakeName) {
    m_shooterTable = NetworkTableInstance.getDefault().getTable(shooterName);
    m_intakeTable = NetworkTableInstance.getDefault().getTable(intakeName);
    setShooterPipeline(0);
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

   public void setPriorityID(double setID) {
    m_shooterTable.getEntry("priorityid").setNumber(setID);
   }

   public double getPriorityID() {
    NetworkTableEntry tid = m_shooterTable.getEntry("priorityid");
    double m_priorityID = tid.getDouble(0);
    return m_priorityID;
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

    public double getDistanceToSpeaker() {
    double[] botPose = getBotPose();
    double distance = new Translation2d().getDistance(new Translation2d(botPose[0], botPose[1]));;
    return distance;
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
    return m_tx;
  }

  public double getNoteTx() {
    NetworkTableEntry tx = m_intakeTable.getEntry("tx");
    double m_tx = tx.getDouble(0.0);
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

  // public void setAllianceColor(DriverStation.Alliance color) {
  //   m_allianceColor = color;
  // }

  public double[] getBotPose() {
    double[] botPose;
      botPose = m_shooterTable.getEntry("botpose_wpiblue").getDoubleArray(new double[] {});

        if (botPose.length >= 18) {
        return botPose;
        }
        return botPose0;
  }

  public double getTxOfTagID(int tagID) {
    return getValueOfTagID(tagID, LimelightConstants.BOTPOSE_RELATIVE_TX_RAW_TARGET_ANGLE);
  }

  public double getDistanceOfTagId(int tagID) {
    return getValueOfTagID(tagID, LimelightConstants.BOTPOSE_RELATIVE_DISTANCE_TO_CAMERA);
  }

  public double getValueOfTagID(int tagID, int value) {
    double[] botPose = getBotPose();
    int numberOfAprilTags = (int) (getBotPoseValue(botPose, LimelightConstants.BOTPOSE_TOTAL_APRILTAGS_SEEN));
    int botPoseAprilTagIndex = LimelightConstants.BOTPOSE_TAG_ID;

    if (numberOfAprilTags > 0) {
      for (int aprilTagIndex = 1; aprilTagIndex < numberOfAprilTags + 1; aprilTagIndex++) {
        if ((int) (getBotPoseValue(botPose, botPoseAprilTagIndex)) == tagID) {
          return getBotPoseValue(botPose, botPoseAprilTagIndex + value);
        } else {
          botPoseAprilTagIndex += LimelightConstants.BOTPOSE_NEXT_INDEX_OF_TAG_ID;
        }
      }
    }
    return 0;
  }

  public double getDistance() {
    double[] targetSpace = m_shooterTable.getEntry("targetpose_robotspace")
        .getDoubleArray(new double[] {});
    // finds distance in meters (needs callibration from limelight) needs to see the
    // april tag (needs testing)
    // might also need to add values like where the limelight is although we might be able to do that in the pipeline

    if (targetSpace.length >= 6) {
      double distance;
      distance = targetSpace[2];

      return distance;
    }
    return 0;
  }

  public double getAngleFromSpeaker() {
    return getAprilTagTx();
  }

  public double getAbsoluteAngleFromSpeaker() {
    double[] botPose = getBotPose();
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      return Math.toDegrees(Math.atan2(getBotPoseValue(botPose, 1) - 5.547, getBotPoseValue(botPose, 0) + 0.381)) + 180;
    } else {
      return Math.toDegrees(Math.atan2(getBotPoseValue(botPose, 1) - 5.547, getBotPoseValue(botPose, 0) - 16.579)) + 180;
    }
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

  public double getBotPoseValue(double[] botPose, int index) {
    if (index < botPose.length) {
      return botPose[index];
    } else {
      return 0.0;
    }
  }

  @Override
  public void periodic() {
    // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
    //   setShooterPipeline(0);
    // }
    // else {
    //   setShooterPipeline(1);
    // }
    
    double[] botPose = getBotPose();
    

      SmartDashboard.putNumber("Limelight/Distance to April Tag", getDistance()); // it calculates in meters
      SmartDashboard.putBoolean("Limelight/Is the target found", getApriltagTargetFound());
      SmartDashboard.putNumber("Limelight/Get April Tag Tx", getAprilTagTx());
      SmartDashboard.putNumber("Limelight/Get Note Tx", getNoteTx());
      SmartDashboard.putNumber("Limelight/Get April Tag Ty", getAprilTagTy());
      SmartDashboard.putNumber("Limelight/Get Note Ty", getNoteTx());
      SmartDashboard.putNumber("Limelight/Get Skew Degree", getSkew_Rotation());
      SmartDashboard.putBoolean("Limelight/April Tag Found", getApriltagTargetFound());

      SmartDashboard.putNumber("Limelight/Get Vertical Degree", getAprilTagTx());

      SmartDashboard.putNumber("Limelight/April Tag IDS", getAprilTagID());
      SmartDashboard.putNumber("Limelight/DISTANCE VALUE", getDistanceOfTagId(4));
      SmartDashboard.putNumber("Limelight/TX VALUE", getTxOfTagID(4));
      SmartDashboard.putNumber("Limelight/Priority ID", getPriorityID());


      SmartDashboard.putNumber("Limelight/ABSOLUTE VALUE OF TX", getAbsoluteAngleFromSpeaker());
      SmartDashboard.putNumber("Limelight/ROBOT RZ", (getBotPoseValue(botPose, 5) < 0 ? getBotPoseValue(botPose, 5)+360 : getBotPoseValue(botPose, 5)));
      
  }
}