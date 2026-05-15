// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder; 

public class IntakeSubsystem extends SubsystemBase {
  
  private final SparkMax m_rollerMotor;
  private final SparkMax m_lifterMotor;
  private final DigitalInput m_lowerLimit;
  private final DigitalInput m_upperLimit;
  private final DutyCycleEncoder m_absoluteEncoder;
  private final DigitalInput m_beamBreak;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(
      SparkMax rollerMotor,
      SparkMax lifterMotor,
      DigitalInput lowerLimit,
      DigitalInput upperLimit,
      DutyCycleEncoder absoluteEncoder,
      DigitalInput beamBreak) {

    m_rollerMotor = rollerMotor;
    m_lifterMotor = lifterMotor;
    m_lowerLimit = lowerLimit;
    m_upperLimit = upperLimit;
    m_absoluteEncoder = absoluteEncoder;
    m_beamBreak = beamBreak;

    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(IdleMode.kBrake).inverted(true);
    m_rollerMotor.configure(rollerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig lifterConfig = new SparkMaxConfig();
    lifterConfig.idleMode(IdleMode.kBrake).inverted(true).smartCurrentLimit(20);
    m_lifterMotor.configure(lifterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_absoluteEncoder.setDutyCycleRange(IntakeConstants.LOWER_PERCENTAGE_ABSOLUTE_ENCODER, IntakeConstants.HIGHER_PERCENTAGE_ABSOLUTE_ENCODER);
  }
  
  public void setBrakeMode(boolean brakeMode) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    m_rollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_lifterMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
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

    if ((atUpperLimit()) && (power > 0)) {
      power = 0;
    }

    if ((atLowerLimit()) && (power < 0)) {
      power = 0;
    }

    m_lifterMotor.set(power);
  }

  /**
   * 
   * @return true if upper limit switch is pressed
   */
  public boolean atUpperLimit() {
    boolean isAtUpper = false;

    if (getAbsoluteEncoderPosition() < IntakeConstants.UPPER_INTAKE_POSITION_VALUE) {
      isAtUpper = true;
    } else {
      isAtUpper = false;
    }

    return isAtUpper;
  }

  public double getAbsoluteEncoderPosition() {
   return m_absoluteEncoder.get() * IntakeConstants.DEGREES_PER_ROTATION;
  }

  /**
   * 
   * @return true if lower limit switch is pressed
   */
  public boolean atLowerLimit() {
    boolean isAtLower = false;

    if (getAbsoluteEncoderPosition() > IntakeConstants.LOWER_INTAKE_POSITION_VALUE) {
      isAtLower = true;
    } else {
      isAtLower = false;
    }
    return isAtLower;
  }

  /** gets beam break */
  public boolean isNoteDetected() {
    return m_beamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // DEBUG 
    SmartDashboard.putBoolean("intake/upper limit", atUpperLimit());
    SmartDashboard.putBoolean("intake/lower limit", atLowerLimit());
    SmartDashboard.putBoolean("intake/beam break", isNoteDetected());
    SmartDashboard.putNumber("intake/rev through bore encoder position", getAbsoluteEncoderPosition());
  }
  
}
