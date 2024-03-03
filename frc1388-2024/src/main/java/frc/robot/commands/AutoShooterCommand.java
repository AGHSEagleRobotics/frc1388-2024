// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;

public class AutoShooterCommand extends Command {
  private final ShooterSubsystem m_shooter;
  private final TransitionSubsystem m_transition;
  private final double SHOOTER_RPM;
  private final Timer m_timer = new Timer();
  private boolean m_noteEverDetected = false;

  /** assumes intake is in up position, ready to shoot */
  public AutoShooterCommand(double rpm, ShooterSubsystem shooter, TransitionSubsystem transition) {
    SHOOTER_RPM = rpm;
    m_shooter = shooter;
    addRequirements(m_shooter);
    m_transition = transition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.setShooterRPM(SHOOTER_RPM);  
    
    if(m_transition.isNoteDetected() == true){
      m_noteEverDetected = true;
      m_timer.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_timer.hasElapsed(0.25) && m_noteEverDetected){
      return true;
    }
    else if( m_timer.hasElapsed(2.5)){
      return true;
    }
    else{
      return false;
    }
  }
}
