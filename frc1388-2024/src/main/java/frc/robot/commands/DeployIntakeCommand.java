// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;

public class DeployIntakeCommand extends Command {

  private final IntakeSubsystem m_intakeSubsystem;
  private final TransitionSubsystem m_transitionSubsystem;
  private int m_ticksNoteIsDetected = 0;
  private Timer m_deployIntakeTimer = new Timer();

  /** Creates a new DeployIntakeCommand. */
  public DeployIntakeCommand(IntakeSubsystem intakeSubsystem, TransitionSubsystem transitionSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_transitionSubsystem = transitionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem, m_transitionSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_deployIntakeTimer.restart();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if (m_deployIntakeTimer.get() < IntakeConstants.LIFTER_MOTOR_TIME_DOWN) {
      m_intakeSubsystem.setLifterMotor(IntakeConstants.LIFTER_MOTOR_SPEED_DOWN);
    } 
    else if (m_deployIntakeTimer.get() < IntakeConstants.LIFTER_MOTOR_TIME_OFF) {
      m_intakeSubsystem.setLifterMotor(0); 
    }
    else if (m_deployIntakeTimer.get() < IntakeConstants.LIFTER_MOTOR_TIME_COAST) {
      m_intakeSubsystem.setBrakeMode(false);
    }
    else {
      m_intakeSubsystem.setBrakeMode(true); 
    } 

    if (m_intakeSubsystem.isNoteDetected()) {
      m_ticksNoteIsDetected++;
      m_intakeSubsystem.setRollerMotor(0);
    } else {
      m_ticksNoteIsDetected = 0;
      m_intakeSubsystem.setRollerMotor(IntakeConstants.ROLLER_MOTOR_SPEED_IN_INTAKING);
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setLifterMotor(0);
    m_intakeSubsystem.setRollerMotor(0);
    m_intakeSubsystem.setBrakeMode(true);
    
    if (!interrupted && m_intakeSubsystem.isNoteDetected()) {
      new RetractIntakeCommand(m_intakeSubsystem, m_transitionSubsystem).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ticksNoteIsDetected > IntakeConstants.TICKS_BEFORE_RETRACTING_INTAKE;
  }
}
