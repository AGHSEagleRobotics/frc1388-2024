// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

public class Eject extends Command {

  private final IntakeSubsystem m_intake;
  private final TransitionSubsystem m_transition;

  /** Creates a new Eject. */
  public Eject(IntakeSubsystem intake, TransitionSubsystem transition) {

    m_intake = intake;
    m_transition = transition;

    addRequirements(m_intake, m_transition);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setRollerMotor(IntakeConstants.ROLLER_MOTOR_SPEED_OUT);
    m_transition.set(TransitionConstants.TRANSITION_MOTOR_POWER_OUT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setRollerMotor(0);
    m_transition.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}