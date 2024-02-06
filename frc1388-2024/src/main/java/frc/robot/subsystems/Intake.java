// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  CANSparkMax m_intake;
  /** Creates a new Intake. */
  public Intake(CANSparkMax intake) {
    m_intake = intake;
  }

  @Override
  public void periodic() {
    m_intake.set(-.65);
    // This method will be called once per scheduler run
  }
}
