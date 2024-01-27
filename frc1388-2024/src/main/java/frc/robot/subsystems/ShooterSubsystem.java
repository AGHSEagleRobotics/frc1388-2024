// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor1;
  private final CANSparkMax m_motor2;
  private final SparkPIDController m_motor1PidController;
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(CANSparkMax motor1, CANSparkMax motor2) {
    m_motor1 = motor1;
    m_motor2 = motor2;
    
    m_motor1.setIdleMode(IdleMode.kCoast);
    m_motor1.setInverted(true);

    m_motor2.setIdleMode(IdleMode.kCoast);
    m_motor2.setInverted(false);

    m_motor1PidController = m_motor1.getPIDController();
  }
    

  public void setPower(double power) {
    m_motor1.set(power);
    m_motor2.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

