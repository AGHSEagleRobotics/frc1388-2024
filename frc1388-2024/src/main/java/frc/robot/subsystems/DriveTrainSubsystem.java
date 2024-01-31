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
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveTrainConstants;

public class DriveTrainSubsystem extends SubsystemBase {

  private Rotation2d m_lastRotation2d = new Rotation2d();

  
  private final SwerveModule m_frontRight, m_frontLeft, m_backLeft, m_backRight;

  private final double ROBOT_LENGTH = Constants.FieldConstants.ROBOT_LENGTH;
  private final double ROBOT_WIDTH = Constants.FieldConstants.ROBOT_WIDTH;

  private final Translation2d m_frontRightTranslation = new Translation2d(ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);
  private final Translation2d m_frontLeftTranslation = new Translation2d(ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
  private final Translation2d m_backLeftTranslation = new Translation2d(-ROBOT_LENGTH / 2, ROBOT_WIDTH / 2);
  private final Translation2d m_backRightTranslation = new Translation2d(-ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2);

  private final Translation2d[] m_swerveTranslation2d = {
    m_frontRightTranslation,
    m_frontLeftTranslation,
    m_backLeftTranslation,
    m_backRightTranslation
  };

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_swerveTranslation2d);
  private SwerveDriveOdometry m_odometry;

  // private final ADIS16470_IMU m_gyro;
  private final AHRS m_navxGyro;

  private ChassisSpeeds m_robotRelativeSpeeds = new ChassisSpeeds();

  public DriveTrainSubsystem(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backLeft, SwerveModule backRight, AHRS gyro) {
    m_frontRight = frontRight;
    m_frontLeft = frontLeft;
    m_backLeft = backLeft;
    m_backRight = backRight;

    m_navxGyro = gyro;
    // m_gyro = gyro;
    
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

      }
    }).start();

    DataLogManager.log("SwerveModuleOffsets: "
        + "  fr: " + Preferences.getDouble(DriveTrainConstants.FRONT_RIGHT_ENCODER_OFFSET_KEY, 0)
        + "  fl: " + Preferences.getDouble(DriveTrainConstants.FRONT_LEFT_ENCODER_OFFSET_KEY, 0)
        + "  bl: " + Preferences.getDouble(DriveTrainConstants.BACK_LEFT_ENCODER_OFFSET_KEY, 0)
        + "  br: " + Preferences.getDouble(DriveTrainConstants.BACK_RIGHT_ENCODER_OFFSET_KEY, 0));
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public void drive(double xVelocity, double yVelocity, double omega) {
    ChassisSpeeds m_robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, omega, getGyroHeading());
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_robotRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveTrainConstants.ROBOT_MAX_SPEED);
    // ask what name the desaturate wheels for constants should be
    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);
    // do the divide by 3 speed here 

    m_odometry.update(
      getGyroHeading(),
      new SwerveModulePosition[] {
          m_frontRight.getPosition(),
          m_frontLeft.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });
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

  private Rotation2d getGyroHeading() {
    if (!m_navxGyro.isCalibrating()) {
      m_lastRotation2d = new Rotation2d(Math.toRadians(Math.IEEEremainder(-m_navxGyro.getAngle(), 360)));
      return m_lastRotation2d;
    } else {
      return m_lastRotation2d;
    }
  }


// for adis delete later
/* 
private Rotation2d getGyroHeading() {
  return new Rotation2d(Math.toRadians(Math.IEEEremainder(m_gyro.getAngle(IMUAxis.kZ), 360)));
}
*/
// replace with above code when we get the navX on 

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
    return m_odometry.getPoseMeters();
  }

  // auto stuff
  private ChassisSpeeds getM_robotRelativeSpeeds() {
    return m_robotRelativeSpeeds;
  }
  
  // auto stuff
  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveTrainConstants.ROBOT_MAX_SPEED);
    //check desaturate constants
    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_frontRight.periodic();
    m_frontLeft.periodic();
    m_backLeft.periodic();
    m_backRight.periodic();
  }
}
