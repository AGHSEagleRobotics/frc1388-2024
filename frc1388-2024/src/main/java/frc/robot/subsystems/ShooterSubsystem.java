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
  private final CANSparkFlex m_motor1;
  private final CANSparkFlex m_motor2;

  private final SparkPIDController m_motor1PidController;
  private final SparkPIDController m_motor2PidController;

  private final RelativeEncoder m_motor1Encoder;
  private final RelativeEncoder m_motor2Encoder;

  private final DataLog m_log = DataLogManager.getLog();
  private final DoubleLogEntry m_logMotor1Velocity = new DoubleLogEntry(m_log, "/robot/motor1Velocity");
  private final DoubleLogEntry m_logMotor2Velocity = new DoubleLogEntry(m_log, "/robot/motor2Velocity");

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(CANSparkFlex motor1, CANSparkFlex motor2) {
    m_motor1 = motor1;
    m_motor2 = motor2;

    m_motor1Encoder = m_motor1.getEncoder();
    m_motor2Encoder = m_motor2.getEncoder();

    m_motor1.setIdleMode(IdleMode.kCoast);
    m_motor1.setInverted(true);

    m_motor2.setIdleMode(IdleMode.kCoast);
    m_motor2.setInverted(false);

    m_motor1PidController = m_motor1.getPIDController();
    m_motor2PidController = m_motor2.getPIDController();

    m_motor1PidController.setP(ShooterSubsystemConstants.kShooterP);
    m_motor1PidController.setI(ShooterSubsystemConstants.kShooterI);
    m_motor1PidController.setD(ShooterSubsystemConstants.kShooterD);
    m_motor1PidController.setFF(ShooterSubsystemConstants.kShooterFF);

    m_motor2PidController.setP(ShooterSubsystemConstants.kShooterP);
    m_motor2PidController.setI(ShooterSubsystemConstants.kShooterI);
    m_motor2PidController.setD(ShooterSubsystemConstants.kShooterD);
    m_motor2PidController.setFF(ShooterSubsystemConstants.kShooterFF);
  }

  public void setPower(double power) {
    m_motor1.set(power);
    m_motor2.set(power);
  }

  /**
   * sets motor 1 velocity
   * 
   * @param rpm setting motor to rpm velocity
   */
  private void setMotor1Velocity(double rpm) {
    m_motor1PidController.setReference(rpm, ControlType.kVelocity);
  }

  private void setMotor2Velocity(double rpm) {
    m_motor2PidController.setReference(rpm, ControlType.kVelocity);
  }

  public double getMotor1Velocity() {
    return m_motor1Encoder.getVelocity();
  }

  public double getMotor2Velocity() {
    return m_motor2Encoder.getVelocity();
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
      m_logMotor1Velocity.append(motor1Velocity);
    }
    double motor2Velocity = getMotor2Velocity();
    if (motor2Velocity != 0) {
      m_logMotor2Velocity.append(motor2Velocity);
    }

  }
}
