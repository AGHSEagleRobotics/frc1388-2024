// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

public class FeedShooter extends Command {
  TransitionSubsystem m_transitionSubsystem;
  IntakeSubsystem m_intakeSubsystem;

  /** Creates a new FeedShooter. */
  public FeedShooter(TransitionSubsystem transitionSubsystem, IntakeSubsystem intakeSubsystem) {
    m_transitionSubsystem = transitionSubsystem;
    m_intakeSubsystem = intakeSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem , m_transitionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.setRollerMotor(IntakeConstants.ROLLER_MOTOR_SPEED_IN);
    m_transitionSubsystem.set(TransitionConstants.TRANSITION_MOTOR_POWER_IN);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setRollerMotor(0);
    m_transitionSubsystem.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
