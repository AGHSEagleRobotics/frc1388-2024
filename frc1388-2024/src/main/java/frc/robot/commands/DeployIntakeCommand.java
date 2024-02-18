// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntakeCommand extends Command {

  private final IntakeSubsystem m_intakeSubsystem;


  /** Creates a new DeployIntakeCommand. */
  public DeployIntakeCommand(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.setRollerMotor(IntakeConstants.ROLLER_MOTOR_SPEED_IN);
    m_intakeSubsystem.setLifterMotor(IntakeConstants.LIFTER_MOTOR_SPEED_DOWN);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setLifterMotor(0);
    m_intakeSubsystem.setRollerMotor(0);
    
    if (!interrupted && m_intakeSubsystem.isNoteDetected()) {
      new RetractIntakeCommand(m_intakeSubsystem).schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.isNoteDetected();
  }
}
