// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LEDSubsystem extends SubsystemBase {

  private final CANdle m_candle; // CANdle canid is 42
  // private final boolean m_isOnRed;

  /** Creates a new LEDSubsystem. */  
  public LEDSubsystem(CANdle candle) {
    m_candle = candle;
    // m_isOnRed = (DriverStation.getAlliance().get() == Alliance.Red);
      CANdleConfiguration config = new CANdleConfiguration();
      config.stripType = LEDStripType.RGB;
      config.brightnessScalar = 0.3; // dim the LEDs to half brightness
      m_candle.configAllSettings(config);
     
      m_candle.clearAnimation(0);
      m_candle.clearAnimation(1);
      m_candle.clearAnimation(2);

    SingleFadeAnimation fades = new SingleFadeAnimation(247, 233, 0, 0, 0.2, 47, 0);
    m_candle.animate(fades);

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
