// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldConstants;

public class DriveTrainSubsystem extends SubsystemBase {

  /** ChassisSpeeds object for the get robot relative speeds method */
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(); 

  /** the SwerveModule objects we created for this class */
  private final SwerveModule m_frontRight, m_frontLeft, m_backLeft, m_backRight;

  /** The distance in <strong>meters</strong> from the center of rotation of the front wheel to the center of rotation of the back wheel */
  private final double ROBOT_WHEEL_BASE = FieldConstants.ROBOT_LENGTH;
  /** The distance in <strong>meters</strong> from the center of rotation of the left wheel to the center of rotation of the right wheel */
  private final double ROBOT_TRACK_WIDTH = FieldConstants.ROBOT_WIDTH;

  // these are the translations from the center of rotation of the robot to the center of rotation of each swerve module
  private final Translation2d m_frontRightTranslation = new Translation2d(ROBOT_WHEEL_BASE / 2, -ROBOT_TRACK_WIDTH / 2);
  private final Translation2d m_frontLeftTranslation = new Translation2d(ROBOT_WHEEL_BASE / 2, ROBOT_TRACK_WIDTH / 2);
  private final Translation2d m_backLeftTranslation = new Translation2d(-ROBOT_WHEEL_BASE / 2, ROBOT_TRACK_WIDTH / 2);
  private final Translation2d m_backRightTranslation = new Translation2d(-ROBOT_WHEEL_BASE / 2, -ROBOT_TRACK_WIDTH / 2);

  /** Translating array for all the swerve modules */
  private final Translation2d[] m_swerveTranslation2d = {
    m_frontRightTranslation,
    m_frontLeftTranslation,
    m_backLeftTranslation,
    m_backRightTranslation
  };

  /** The kinematics object does all the swerve math */
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_swerveTranslation2d);
  /** The odometry object keeps track of the robots position */
  private SwerveDriveOdometry m_odometry;

  /** gyro for detecting rotation angle */
  private final AHRS m_navxGyro;

  public DriveTrainSubsystem(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backLeft, SwerveModule backRight, AHRS gyro) {

    m_frontRight = frontRight;
    m_frontLeft = frontLeft;
    m_backLeft = backLeft;
    m_backRight = backRight;

    m_navxGyro = gyro;
    
    //gyro and odometry setup code I copied from a youtube video <br> https://www.youtube.com/watch?v=0Xi9yb1IMyA
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        m_navxGyro.reset();
        m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            getGyroHeading(),
            new SwerveModulePosition[] {
                m_frontRight.getPosition(),
                m_frontLeft.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            },
            new Pose2d(0, 0, new Rotation2d()));
      } catch (Exception e) {
        // e.printStackTrace();
        System.out.println(e.toString());
      }
    }).start();

    DataLogManager.log("SwerveModuleOffsets: "
        + "  fr: " + Preferences.getDouble(DriveTrainConstants.FRONT_RIGHT_ENCODER_OFFSET_KEY, 0)
        + "  fl: " + Preferences.getDouble(DriveTrainConstants.FRONT_LEFT_ENCODER_OFFSET_KEY, 0)
        + "  bl: " + Preferences.getDouble(DriveTrainConstants.BACK_LEFT_ENCODER_OFFSET_KEY, 0)
        + "  br: " + Preferences.getDouble(DriveTrainConstants.BACK_RIGHT_ENCODER_OFFSET_KEY, 0));

  }

  /** returns the kinematics object */
  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /** the drive method takes in an x and y velocity in meters / second, and a rotation rate in radians / second */
  public void drive(double xVelocity, double yVelocity, double omega) {
    ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, omega, getGyroHeading());
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(robotRelativeSpeeds);

    // optimises wheel heading direction changes.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrainConstants.ROBOT_MAX_SPEED);
    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);
    // do the divide by 3 speed here 

    // odometry updating
    if (m_odometry != null) {
      m_odometry.update(
        getGyroHeading(),
        new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }
      );
    }
  }
    /**
     * Applies offset to ALL swerve modules.
     * <p>
     * Wheels MUST be pointed to 0 degrees relative to robot to use this method
     */
  public void setAllEncoderOffsets(){
    double frontRightOffset = m_frontRight.setEncoderOffset();
    double frontLeftOffset = m_frontLeft.setEncoderOffset();
    double backLeftOffset = m_backLeft.setEncoderOffset();
    double backRightOffset = m_backRight.setEncoderOffset();

    Preferences.setDouble(DriveTrainConstants.FRONT_RIGHT_ENCODER_OFFSET_KEY, frontRightOffset);
    Preferences.setDouble(DriveTrainConstants.FRONT_LEFT_ENCODER_OFFSET_KEY, frontLeftOffset);
    Preferences.setDouble(DriveTrainConstants.BACK_LEFT_ENCODER_OFFSET_KEY, backLeftOffset);
    Preferences.setDouble(DriveTrainConstants.BACK_RIGHT_ENCODER_OFFSET_KEY, backRightOffset);

    DataLogManager.log("SwerveModuleUpdatedOffsets: " + "setting offsets: "
        + "  fr: " + frontRightOffset
        + "  fl: " + frontLeftOffset
        + "  bl: " + backLeftOffset
        + "  br: " + backRightOffset);
  }

  public Rotation2d getGyroHeading() {
    if (!m_navxGyro.isCalibrating()) {
      return new Rotation2d(Math.toRadians(Math.IEEEremainder(-m_navxGyro.getAngle(), 360)));
    }
    return new Rotation2d();
  }

  public void resetGyroHeading() {
    m_navxGyro.reset();
  }

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(getGyroHeading(),
        new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        },
        pose);
  }

  public Pose2d getPose() {
    if (m_odometry != null) {
      return m_odometry.getPoseMeters();
    }

    // return new Pose2d(123, 432, m_lastRotation2D);
    return new Pose2d(123, 432, getGyroHeading());
  }


  // XXX test me (while testing other method with this note)
  // temporarily made public
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return chassisSpeeds;
  }

  // // XXX test me (while testing other method with this note)
  // // temporarily made public
  // public void driveRobotRelative(ChassisSpeeds speeds) {
  //   // double xvel = speeds.vxMetersPerSecond;
  //   // double yvel = speeds.vyMetersPerSecond;

  //   // speeds.vxMetersPerSecond = yvel;
  //   // speeds.vyMetersPerSecond = -xvel;

  //   SmartDashboard.putNumber("auto y speed", speeds.vyMetersPerSecond);
  //   SmartDashboard.putNumber("auto x speed", speeds.vxMetersPerSecond);

  //   chassisSpeeds = speeds;
  
  // auto stuff
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveTrainConstants.ROBOT_MAX_SPEED);
    //check desaturate constants
    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);


    if (m_odometry != null) {
      m_odometry.update(
        getGyroHeading(),
        new SwerveModulePosition[] {
            m_frontRight.getPosition(),
            m_frontLeft.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
        // System.out.println("odo is being updated");
    }
  }

  public double getDistTraveled() {
    return m_frontRight.getPosition().distanceMeters;
  }

  public double getAngle() {
    if (!m_navxGyro.isCalibrating()) {
      return (-m_navxGyro.getAngle() + 36000) % 360;
    }
    return 0;
  }

  public void setWheelAngle(double angle) {
    m_frontRight.setRotationPosition(angle);
    m_frontLeft.setRotationPosition(angle);
    m_backLeft.setRotationPosition(angle);
    m_backRight.setRotationPosition(angle);
  }

  public void differentialDrive(double speed) {
    m_frontRight.setRotationPosition(0);
    m_frontLeft.setRotationPosition(0);
    m_backLeft.setRotationPosition(0);
    m_backRight.setRotationPosition(0);

    m_frontRight.setDriveSpeed(speed);
    m_frontLeft.setDriveSpeed(speed);
    m_backLeft.setDriveSpeed(speed);
    m_backRight.setDriveSpeed(speed);
  }

  public void setBrakeMode(boolean brakeMode){
    m_frontRight.setBrakeMode(brakeMode);
    m_frontLeft.setBrakeMode(brakeMode);
    m_backLeft.setBrakeMode(brakeMode);
    m_backRight.setBrakeMode(brakeMode);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_frontRight.periodic();
    m_frontLeft.periodic();
    m_backLeft.periodic();
    m_backRight.periodic();
    SmartDashboard.putNumber("odo x", getPose().getX());
    SmartDashboard.putNumber("odo y", getPose().getY());

    SmartDashboard.putString("auto speeds", getRobotRelativeSpeeds().toString());

    // System.out.println("is odo null?" + (m_odometry == null));

    SmartDashboard.putNumber("gyro angle", getAngle());

  }
}
