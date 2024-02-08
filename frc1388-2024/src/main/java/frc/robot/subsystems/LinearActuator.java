// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearActuator extends SubsystemBase {
  CANSparkMax m_laMotor = new CANSparkMax(33, MotorType.kBrushed);
  AnalogPotentiometer m_pot = new AnalogPotentiometer(3);
  double m_motorPower = 0;

  /** Creates a new LinearActuator. */
  public LinearActuator() {
  }

  public void setMotor(double power) {
    m_motorPower = power;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_laMotor.set(m_motorPower);
    m_motorPower = 0;

    SmartDashboard.putNumber("LinearActuatorPosition", m_pot.get());
  }
}
