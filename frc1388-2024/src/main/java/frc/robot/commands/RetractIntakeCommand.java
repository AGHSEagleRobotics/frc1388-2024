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
  private final boolean m_endRetractFlag;

  private double m_intakeSpeed = IntakeConstants.LIFTER_MOTOR_SPEED_UP;

  public RetractIntakeCommand(IntakeSubsystem intakeSubsystem, TransitionSubsystem transitionSubsystem, boolean endRectractFlag) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSubsystem = intakeSubsystem;
    m_transitionSubsystem = transitionSubsystem;
    m_endRetractFlag = endRectractFlag;

    addRequirements(m_intakeSubsystem, m_transitionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (! m_intakeSubsystem.atUpperLimit()) {
      m_intakeSubsystem.setLifterMotor(m_intakeSpeed);
    }
    else {
      m_intakeSubsystem.setLifterMotor(0);
      m_intakeSpeed = IntakeConstants.LIFTER_MOTOR_SPEED_UP_HOLD;
    }

    m_intakeSubsystem.setRollerMotor(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.setLifterMotor(0);
    m_intakeSubsystem.setRollerMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_intakeSubsystem.atUpperLimit() && (m_endRetractFlag)) {
      return true;
    }
    return false;
  }
}
