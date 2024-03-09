// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterAngleSubsystemConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class AutoShooterAngle extends Command {
private final ShooterAngleSubsystem m_shooterAngleSubsystem;
private double m_shooterAngle;

  /** Creates a new ShooterAngleCommand. */
  public AutoShooterAngle(double shooterAngle, ShooterAngleSubsystem shooterAngleSubsystem) { 
    m_shooterAngleSubsystem = shooterAngleSubsystem;
    m_shooterAngle = shooterAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterAngleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterAngleSubsystem.setPosition(m_shooterAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if( Math.abs(m_shooterAngle - m_shooterAngleSubsystem.getCurrentPosition()) < ShooterAngleSubsystemConstants.P_TOLERANCE){
      return true;
    }
    else{
      return false;
    }
  }
}
