// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private final PWMSparkMax m_led;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(PWMSparkMax led) {
    m_led = led;
  }

  @Override
  public void periodic() {
    m_led.set(-0.99);
    // This method will be called once per scheduler run
  }
}
