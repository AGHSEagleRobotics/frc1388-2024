// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterCommandConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

public class ShooterCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final Timer m_timer = new Timer();
  
  /** assumes intake is in up position, ready to shoot */
  public ShooterCommand(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_shooter.setPower(ShooterCommandConstants.kInitialMotorSpeed);
    m_shooter.setShooterRPM(3000);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(ShooterCommandConstants.kShooterTimer);
  }
}