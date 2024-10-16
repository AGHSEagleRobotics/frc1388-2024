// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LEDSubsystem extends SubsystemBase {

  private final CANdle m_candle;
  private final PowerDistribution m_PowerDistribution;
  // private final boolean m_isOnRed;
  private int m_loopCount = 0;

  private final int LOOPS_PER_SECOND = 50;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(CANdle candle, PowerDistribution powerDistribution) {
    m_candle = candle;
    m_PowerDistribution = powerDistribution;

    // m_isOnRed = (DriverStation.getAlliance().get() == Alliance.Red);
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.1; // dim the LEDs to half brightness
    m_candle.configAllSettings(config);

    LED_reset();
  }

  /** Reset internal and external LEDs - turn off LEDs and animations */
  public void LED_reset() {
    // Turn off LEDs
    m_candle.setLEDs(0, 0, 0);

    // clear all animations
    for (int slot = 0; slot < m_candle.getMaxSimultaneousAnimationCount(); slot++) {
      m_candle.clearAnimation(slot);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_loopCount++;

    // Once per second:
    if ((m_loopCount % LOOPS_PER_SECOND) == 0) {

      // set alliance color
      Alliance alliance_color = DriverStation.getAlliance().get();
      if (alliance_color == Alliance.Red) {
        m_candle.setLEDs(255, 0, 0, 0, 4, 4);
      }
      else if (alliance_color == Alliance.Blue) {
        m_candle.setLEDs(0, 0, 255, 0, 4, 4);
      }
      else {
        m_candle.setLEDs(0, 0, 0, 0, 4, 4);
      }
    }
    
    // Battery meter
    {
      final double VBAT_CRITICAL = 12.2;
      final double VBAT_LOW = 12.4;
      final double VBAT_OK = 12.6;

      // Set LEDs
      double vBat = m_PowerDistribution.getVoltage();

      if (vBat < VBAT_CRITICAL) {
        m_candle.setLEDs(255, 0, 0, 0, 0, 1);
        m_candle.setLEDs(0, 0, 0, 0, 1, 3);
      } else if (vBat < VBAT_LOW) {
        m_candle.setLEDs(255, 255, 0, 0, 0, 2);
        m_candle.setLEDs(0, 0, 0, 0, 2, 2);
      } else if (vBat < VBAT_OK) {
        m_candle.setLEDs(0, 255, 0, 0, 0, 3);
        m_candle.setLEDs(0, 0, 0, 0, 3, 1);
      } else if (vBat < VBAT_OK) {
        m_candle.setLEDs(0, 255, 0, 0, 0, 4);
      }
    }
  }
}
