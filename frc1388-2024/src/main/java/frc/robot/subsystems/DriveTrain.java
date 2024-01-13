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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

public class DriveTrain extends SubsystemBase {

  private Rotation2d lastRotation2d = new Rotation2d();

  // XXX I think this is right, but if the robot drives funny, check these numbers
  private final double ROBOT_WIDTH = 0.508;
  private final double ROBOT_LENGTH = 0.508;

  private final SwerveModule m_frontRight, m_frontLeft, m_backLeft, m_backRight;

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

  private final AHRS m_navxGyro;

  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

  public DriveTrain(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backLeft, SwerveModule backRight, AHRS gyro) {
    m_frontRight = frontRight;
    m_frontLeft = frontLeft;
    m_backLeft = backLeft;
    m_backRight = backRight;

    m_navxGyro = gyro;
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

  public void setModuleState(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0);

    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);
  }

  public void drive(double xVelocity, double yVelocity, double omega) {
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, omega, getGyroHeading());
    robotRelativeSpeeds = fieldRelativeSpeeds;
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(fieldRelativeSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0);
    
    m_frontRight.setSwerveModuleStates(states[0]);
    m_frontLeft.setSwerveModuleStates(states[1]);
    m_backLeft.setSwerveModuleStates(states[2]);
    m_backRight.setSwerveModuleStates(states[3]);

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
      lastRotation2d = new Rotation2d(Math.toRadians(Math.IEEEremainder(-m_navxGyro.getAngle(), 360)));
      return lastRotation2d;
    } else {
      return lastRotation2d;
    }
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
    return m_odometry.getPoseMeters();
  }

  // auto stuff
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }
  
  // auto stuff
  private void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.0);

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
