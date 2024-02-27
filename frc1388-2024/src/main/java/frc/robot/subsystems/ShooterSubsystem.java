// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkFlex m_bottomShooterMotor;
  private final CANSparkFlex m_topShooterMotor;

  private final SparkPIDController m_bottomShooterMotorPIDController;
  private final SparkPIDController m_topShooterMotorPIDController;

  private final RelativeEncoder m_bottomShooterEncoder;
  private final RelativeEncoder m_topMotorEncoder;

  private final DataLog m_log = DataLogManager.getLog();
  private final DoubleLogEntry m_logBottomShooterMotorVelocity = new DoubleLogEntry(m_log, "/robot/bottomShooterMotorVelocity");
  private final DoubleLogEntry m_logTopShooterMotorVelocity = new DoubleLogEntry(m_log, "/robot/topShooterMotorVelocity");

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(CANSparkFlex bottomShooterMotor, CANSparkFlex topShooterMotor) {
    m_bottomShooterMotor = bottomShooterMotor;
    m_topShooterMotor = topShooterMotor;

    m_bottomShooterEncoder = m_bottomShooterMotor.getEncoder();
    m_topMotorEncoder = m_topShooterMotor.getEncoder();

    m_bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    m_bottomShooterMotor.setInverted(true);

    m_topShooterMotor.setIdleMode(IdleMode.kCoast);
    m_topShooterMotor.setInverted(false);

    m_bottomShooterMotorPIDController = m_bottomShooterMotor.getPIDController();
    m_topShooterMotorPIDController = m_topShooterMotor.getPIDController();

    m_bottomShooterMotorPIDController.setP(ShooterConstants.SHOOTER_MOTOR_P);
    m_bottomShooterMotorPIDController.setI(ShooterConstants.SHOOTER_MOTOR_I);
    m_bottomShooterMotorPIDController.setD(ShooterConstants.SHOOTER_MOTOR_D);
    m_bottomShooterMotorPIDController.setFF(ShooterConstants.SHOOTER_MOTOR_FF);

    m_topShooterMotorPIDController.setP(ShooterConstants.SHOOTER_MOTOR_P);
    m_topShooterMotorPIDController.setI(ShooterConstants.SHOOTER_MOTOR_I);
    m_topShooterMotorPIDController.setD(ShooterConstants.SHOOTER_MOTOR_D);
    m_topShooterMotorPIDController.setFF(ShooterConstants.SHOOTER_MOTOR_FF);
  }

  public void setPower(double power) {
    m_bottomShooterMotor.set(power);
    m_topShooterMotor.set(power);
  }

  /**
   * sets bottom motor velocity
   * 
   * @param rpm setting motor to rpm velocity
   */
  private void setBottomMotorVelocity(double rpm) {
    m_bottomShooterMotorPIDController.setReference(rpm, ControlType.kVelocity);
  }

    /**
   * sets top motor velocity
   * 
   * @param rpm setting motor to rpm velocity
   */
  private void setTopMotorVelocity(double rpm) {
    m_topShooterMotorPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public double getBottomMotorVelocity() {
    return m_bottomShooterEncoder.getVelocity();
  }

  public double getTopMotorVelocity() {
    return m_topMotorEncoder.getVelocity();
  }

  public void setShooterRPM(double rpm) {
    setBottomMotorVelocity(rpm);
    setTopMotorVelocity(rpm);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double bottomMotorVelocity = getBottomMotorVelocity();
    double topMotorVelocity = getTopMotorVelocity();

    SmartDashboard.putNumber("bottom shooter rpm", bottomMotorVelocity);
    SmartDashboard.putNumber("top shooter rpm", topMotorVelocity);

    if (bottomMotorVelocity != 0) {
      m_logBottomShooterMotorVelocity.append(bottomMotorVelocity);
    }
    if (topMotorVelocity != 0) {
      m_logTopShooterMotorVelocity.append(topMotorVelocity);
    }

  }
}
