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

public class TransitionSubsystem extends SubsystemBase {

  private final SparkMax m_transitionMotor;
  private final DigitalInput m_beamBreak;

  /** Creates a new TransitionSubsystem. */
  public TransitionSubsystem(SparkMax transitionMotor, DigitalInput beamBreak) {
    m_transitionMotor = transitionMotor;
    m_transitionMotor.setInverted(false);

    m_beamBreak = beamBreak;
  }

  public void setBrakeMode(boolean brakeMode) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    m_transitionMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**Set transition speed, positive is into shooter
   * @param speed [-1, 1]
   */
  public void set(double speed) {
    m_transitionMotor.set(speed);
  }

  public boolean isNoteDetected() {
    return m_beamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("transition subsystem/beam break", isNoteDetected());

  }
}
