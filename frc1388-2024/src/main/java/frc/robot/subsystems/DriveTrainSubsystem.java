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
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class DriveTrainSubsystem extends SubsystemBase {

  private Rotation2d m_lastRotation2d = new Rotation2d();

  
  private final SwerveModule m_frontRight, m_frontLeft, m_backLeft, m_backRight;

  private final Translation2d m_frontRightTranslation = new Translation2d(
      Constants.FieldConstants.ROBOT_LENGTH / 2, -Constants.FieldConstants.ROBOT_WIDTH / 2);
  private final Translation2d m_frontLeftTranslation = new Translation2d(Constants.FieldConstants.ROBOT_LENGTH / 2,
      Constants.FieldConstants.ROBOT_WIDTH / 2);
  private final Translation2d m_backLeftTranslation = new Translation2d(-Constants.FieldConstants.ROBOT_LENGTH / 2,
      Constants.FieldConstants.ROBOT_WIDTH / 2);
  private final Translation2d m_backRightTranslation = new Translation2d(
      -Constants.FieldConstants.ROBOT_LENGTH / 2, -Constants.FieldConstants.ROBOT_WIDTH / 2);

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
  }

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }
  // XXX ask Calvin what this is used for
/* 
  public void setModuleState(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0);

    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);
  }
*/
  public void drive(double xVelocity, double yVelocity, double omega) {
    ChassisSpeeds m_robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, omega, getGyroHeading());
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_robotRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0);
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
  private void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0);
    //check destatureate constants
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
