// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TransitionSubsystem extends SubsystemBase {

  private final CANSparkMax m_transitionMotor;
  private final DigitalInput m_beamBreak;

  /** Creates a new TransitionSubsystem. */
  public TransitionSubsystem(CANSparkMax transitionMotor, DigitalInput beamBreak) {
    m_transitionMotor = transitionMotor;
    m_transitionMotor.setInverted(false);

    m_beamBreak = beamBreak;
  }

  public void setBrakeMode(boolean brakeMode) {
    if (brakeMode) {
      m_transitionMotor.setIdleMode(IdleMode.kBrake);
    } else {
      m_transitionMotor.setIdleMode(IdleMode.kCoast);
    }
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
  }
}
