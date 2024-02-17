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
import frc.robot.Constants.ShooterSubsystemConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkFlex m_bottomShooterMotor;
  private final CANSparkFlex m_topShooterMotor;

  private final SparkPIDController m_bottomShooterMotorPIDController;
  private final SparkPIDController m_topShooterMotorPIDController;

  private final RelativeEncoder m_bottomShooterMotorEncoder;
  private final RelativeEncoder m_topShooterMotorEncoder;

  private final DataLog m_log = DataLogManager.getLog();
  private final DoubleLogEntry m_logBottomShooterMotorVelocity = new DoubleLogEntry(m_log, "/robot/bottomShooterMotorVelocity");
  private final DoubleLogEntry m_logTopShooterMotorVelocity = new DoubleLogEntry(m_log, "/robot/topShooterMotorVelocity");

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(CANSparkFlex bottomShooterMotor, CANSparkFlex topShooterMotor) {
    m_bottomShooterMotor = bottomShooterMotor;
    m_topShooterMotor = topShooterMotor;

    m_bottomShooterMotorEncoder = m_bottomShooterMotor.getEncoder();
    m_topShooterMotorEncoder = m_topShooterMotor.getEncoder();

    m_bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    m_bottomShooterMotor.setInverted(true);

    m_topShooterMotor.setIdleMode(IdleMode.kCoast);
    m_topShooterMotor.setInverted(false);

    m_bottomShooterMotorPIDController = m_bottomShooterMotor.getPIDController();
    m_topShooterMotorPIDController = m_topShooterMotor.getPIDController();

    m_bottomShooterMotorPIDController.setP(ShooterSubsystemConstants.kShooterP);
    m_bottomShooterMotorPIDController.setI(ShooterSubsystemConstants.kShooterI);
    m_bottomShooterMotorPIDController.setD(ShooterSubsystemConstants.kShooterD);
    m_bottomShooterMotorPIDController.setFF(ShooterSubsystemConstants.kShooterFF);

    m_topShooterMotorPIDController.setP(ShooterSubsystemConstants.kShooterP);
    m_topShooterMotorPIDController.setI(ShooterSubsystemConstants.kShooterI);
    m_topShooterMotorPIDController.setD(ShooterSubsystemConstants.kShooterD);
    m_topShooterMotorPIDController.setFF(ShooterSubsystemConstants.kShooterFF);
  }

  public void setPower(double power) {
    m_bottomShooterMotor.set(power);
    m_topShooterMotor.set(power);
  }

  /**
   * sets motor 1 velocity
   * 
   * @param rpm setting motor to rpm velocity
   */
  private void setMotor1Velocity(double rpm) {
    m_bottomShooterMotorPIDController.setReference(rpm, ControlType.kVelocity);
  }

  private void setMotor2Velocity(double rpm) {
    m_topShooterMotorPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public double getMotor1Velocity() {
    return m_bottomShooterMotorEncoder.getVelocity();
  }

  public double getMotor2Velocity() {
    return m_topShooterMotorEncoder.getVelocity();
  }

  public void setShooterRPM(double rpm) {
    setMotor1Velocity(rpm);
    setMotor2Velocity(rpm);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //Logging motor velocity
    double motor1Velocity = getMotor1Velocity();
    if (motor1Velocity != 0) {
      m_logBottomShooterMotorVelocity.append(motor1Velocity);
    }
    double motor2Velocity = getMotor2Velocity();
    if (motor2Velocity != 0) {
      m_logTopShooterMotorVelocity.append(motor2Velocity);
    }

  }
}
