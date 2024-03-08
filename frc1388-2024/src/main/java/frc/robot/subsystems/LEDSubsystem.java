// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {

  private final PWMSparkMax m_led;
  // private final boolean m_isOnRed;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(PWMSparkMax led) {
    m_led = led;
    // m_isOnRed = (DriverStation.getAlliance().get() == Alliance.Red);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (m_isOnRed) {
    //   m_led.set(LEDConstants.RED_SOLID);
    // } else {
    //   m_led.set(LEDConstants.BLUE_SOLID);
    // }
  }
}
