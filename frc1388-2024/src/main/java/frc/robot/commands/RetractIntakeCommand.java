// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

/** Creates a new RetractIntakeCommand. */
public class RetractIntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final TransitionSubsystem m_transitionSubsystem;

  public RetractIntakeCommand(IntakeSubsystem intakeSubsystem, TransitionSubsystem transitionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSubsystem = intakeSubsystem;
    m_transitionSubsystem = transitionSubsystem;
    addRequirements(m_intakeSubsystem, m_transitionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO increase lifter motor speed
    m_intakeSubsystem.setLifterMotor(IntakeConstants.LIFTER_MOTOR_SPEED_UP);
    m_intakeSubsystem.setRollerMotor(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if (!interrupted) {
    //   new PullToTransition(m_transitionSubsystem, m_intakeSubsystem).schedule();
    // }
    m_intakeSubsystem.setLifterMotor(0);
    m_intakeSubsystem.setRollerMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.getUpperLimit();
  }
}
