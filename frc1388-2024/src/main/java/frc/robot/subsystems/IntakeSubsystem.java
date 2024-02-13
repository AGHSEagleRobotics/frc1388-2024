// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_rollerMotor;
  private final CANSparkMax m_lifterMotor;
  private final DigitalInput m_lowerLimit;
  private final DigitalInput m_upperLimit;
  private final DigitalInput m_beamBreak;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(
      CANSparkMax rollerMotor,
      CANSparkMax lifterMotor,
      DigitalInput lowerLimit,
      DigitalInput upperLimit,
      DigitalInput beamBreak) {

    m_rollerMotor = rollerMotor;
    m_lifterMotor = lifterMotor;
    m_lowerLimit = lowerLimit;
    m_upperLimit = upperLimit;
    m_beamBreak = beamBreak;

    m_rollerMotor.setIdleMode(IdleMode.kBrake);
    m_rollerMotor.setInverted(true);

    m_lifterMotor.setIdleMode(IdleMode.kBrake);
    m_lifterMotor.setInverted(true);
  }

  /**
   * sets power to intake roller motor
   * 
   * @param power value of -1.0 to 1.0, positive is intake
   */

  public void setRollerMotor(double power) {
    m_rollerMotor.set(power);
  }

  /**
   * sets power to lifter motor
   * when either switch is pressed the motor stops
   * 
   * @param power value of -1.0 to 1.0, positive is up
   */
  public void setLifterMotor(double power) {

    if ((getUpperLimit()) && (power > 0)) {
      power = 0;
    }

    if ((getLowerLimit()) && (power < 0)) {
      power = 0;
    }

    m_lifterMotor.set(power);
  }

  /**
   * 
   * @return true if upper limit switch is pressed
   */
  public boolean getUpperLimit() {

    return !m_upperLimit.get();

  }

  /**
   * 
   * @return true if lower limit switch is pressed
   */
  public boolean getLowerLimit() {

    return !m_lowerLimit.get();

  }

  /** gets beam break */
  public boolean getBeamBreak() {
    return m_beamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("upper limit", getUpperLimit());
    SmartDashboard.putBoolean("lower limit", getLowerLimit());
    SmartDashboard.putBoolean("beam break", getBeamBreak());
    m_rollerMotor.set(0.0);
  }
}
