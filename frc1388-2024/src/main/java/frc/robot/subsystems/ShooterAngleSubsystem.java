// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterAngleSubsystemConstants;
import frc.robot.Constants.ShooterSubsystemConstants;


public class ShooterAngleSubsystem extends SubsystemBase {
  private final CANSparkMax m_angleMotor;
  private final AnalogPotentiometer m_potentiometer;
  private final SparkPIDController m_angleMotorPIDController;

  /** Creates a new ShooterAngleSubsystem. */
  public ShooterAngleSubsystem(CANSparkMax angleMotor, AnalogPotentiometer potentiometer) {
    m_angleMotor = angleMotor;
    m_potentiometer = potentiometer;
    m_angleMotorPIDController = m_angleMotor.getPIDController();

    m_angleMotorPIDController.setP(ShooterAngleSubsystemConstants.kShooterAngleP);
    m_angleMotorPIDController.setI(ShooterAngleSubsystemConstants.kShooterAngleI);
    m_angleMotorPIDController.setD(ShooterAngleSubsystemConstants.kShooterAngleD);
    m_angleMotorPIDController.setFF(ShooterAngleSubsystemConstants.kShooterAngleFF);    
  }

  public void setPower(double power) {
    m_angleMotor.set(power);
  }

  public double getPotentiometer() {
    return m_potentiometer.get();
  }

  public void setPosition(double position) {
    if (position > ShooterAngleSubsystemConstants.kShooterPositionMax) {
      position = ShooterAngleSubsystemConstants.kShooterPositionMax;
    }
    if (position < ShooterAngleSubsystemConstants.kShooterPositionMin) {
      position = ShooterAngleSubsystemConstants.kShooterPositionMin;
    }
    
    m_angleMotorPIDController.setReference(position, ControlType.kPosition);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("potentiometerPosition", getPotentiometer());
    // This method will be called once per scheduler run
  }
}
