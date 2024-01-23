// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor1;
  private final CANSparkMax m_motor2;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(CANSparkMax motor1, CANSparkMax motor2) {
    m_motor1 = motor1;
    m_motor2 = motor2;
    motor1.setIdleMode(IdleMode.kCoast);
    m_motor1.setInverted(true);
    motor2.setIdleMode(IdleMode.kCoast);
    m_motor2.setInverted(false);
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
