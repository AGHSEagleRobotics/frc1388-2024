// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterAngleSubsystemConstants;


public class ShooterAngleSubsystem extends SubsystemBase {
  private final CANSparkMax m_angleMotor;
  private final AnalogPotentiometer m_potentiometer;
  private double m_targetPosition;

  /** Creates a new ShooterAngleSubsystem. */
  public ShooterAngleSubsystem(CANSparkMax angleMotor, AnalogPotentiometer potentiometer) {
    m_angleMotor = angleMotor;
    m_potentiometer = potentiometer;
    m_targetPosition = getCurrentPosition();
  }

  public double getCurrentPosition() {
    return m_potentiometer.get();
  }

  public void setPosition(double position) {
    m_targetPosition = position;
  }

  @Override
  public void periodic() {
    m_targetPosition = MathUtil.clamp(m_targetPosition, ShooterAngleSubsystemConstants.kShooterMinHeight, ShooterAngleSubsystemConstants.kShooterMaxHeight);
    
    double currentPosition = getCurrentPosition();
    double error = m_targetPosition - currentPosition;
    double speed = error * ShooterAngleSubsystemConstants.kShooterAngleP;
    SmartDashboard.putNumber("error", error);
    if (Math.abs(error) <= ShooterAngleSubsystemConstants.P_TOLERANCE) {
      speed = 0;
    }
    m_angleMotor.set(speed);


    // if (currentPosition < m_targetPosition - 0.01) {
    // m_angleMotor.set(1);
    // }
    // else if (currentPosition > m_targetPosition + 0.01) {
    //   m_angleMotor.set(-1);
    // }
    SmartDashboard.putNumber("potentiometerPosition", currentPosition);
    SmartDashboard.putNumber("target position", m_targetPosition);
    SmartDashboard.putNumber("speed", speed);
    // This method will be called once per scheduler run
  }
}
