// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import au.grapplerobotics.LaserCan;

public class LEDSubsystem extends SubsystemBase {

  private final CANdle m_candle; //candle id 42

  StrobeAnimation strobeAnim = new StrobeAnimation(100, 0, 0, 0, 0.7, 47, 0);
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem(CANdle candle) {
    m_candle = candle;
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.1; //dim leds to half brightness
    m_candle.configAllSettings(config);

  }

  @Override
  public void periodic() {
    LaserCan.Measurement measurement = m_lasercan.getMeasurement();

    if (LidarSubsystem().measurement >= 100) {
      m_candle.animate(strobeAnim);
    }
    // This method will be called once per scheduler run
    // if (m_isOnRed) {
    //   m_led.set(LEDConstants.RED_SOLID);
    // } else {
    //   m_led.set(LEDConstants.BLUE_SOLID);
    // }
  }
}
