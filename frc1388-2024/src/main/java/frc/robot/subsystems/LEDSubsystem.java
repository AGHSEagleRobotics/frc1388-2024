// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

  private final PWMSparkMax m_leds;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(PWMSparkMax led) {
    m_leds = led;
  }

  public void setRed() {
    m_leds.set(LEDConstants.RED);
  }

  public void setGreen() {
    m_leds.set(LEDConstants.GREEN_RAINBOW);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
