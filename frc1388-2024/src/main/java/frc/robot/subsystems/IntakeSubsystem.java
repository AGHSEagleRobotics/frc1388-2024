// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_rollerMotor; 
  private final CANSparkMax m_lifterMotor;
  private final DigitalInput m_lowerLimit;
  private final DigitalInput m_upperLimit;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(CANSparkMax rollerMotor, 
                          CANSparkMax lifterMotor, 
                         DigitalInput lowerLimit,
                         DigitalInput upperLimit) {
    
    m_rollerMotor = rollerMotor;
    m_lifterMotor = lifterMotor;
    m_lowerLimit = lowerLimit;
    m_upperLimit = upperLimit;

    m_rollerMotor.setIdleMode(IdleMode.kBrake);
    m_rollerMotor.setInverted(true);

    m_lifterMotor.setIdleMode(IdleMode.kBrake);
    m_lifterMotor.setInverted(true);



  }

  /**
   * sets power to intake roller motor
   * @param power value of -1.0 to 1.0, positive is intake 
   */

  public void setRollerMotor(double power) {

    m_rollerMotor.set(power);

  }

  /**
   * sets power to lifter motor
   * @param power value of -1.0 to 1.0, positive is up
   */
  public void setLifterMotor(double power){

    m_lifterMotor.set(power);

  }

  public boolean getUpperLimit(){
    
    return m_upperLimit.get();

  } 

  public boolean getLowerLimit(){

    return m_lowerLimit.get();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run



  }
}
