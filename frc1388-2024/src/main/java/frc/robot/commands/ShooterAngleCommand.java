// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterAngleSubsystemConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterAngleCommand extends Command {

  private final Supplier<Boolean> m_dPadUp;
  private final Supplier<Boolean> m_dPadDown;
  private final ShooterAngleSubsystem m_ShooterAngleSubsystem;

  /** Creates a new ShooterAngleCommand. */
  public ShooterAngleCommand(Supplier<Boolean> dPadUp, Supplier<Boolean> dPadDown, ShooterAngleSubsystem ShooterAngleSubsystem) {
    m_dPadUp = dPadUp;
    m_dPadDown = dPadDown;
    m_ShooterAngleSubsystem = ShooterAngleSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ShooterAngleSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_dPadUp.get() ) {
      m_ShooterAngleSubsystem.setPosition(ShooterAngleSubsystemConstants.kShooterPositionUp);
    }
    else if(m_dPadDown.get() ){
      m_ShooterAngleSubsystem.setPosition(ShooterAngleSubsystemConstants.kShooterPositionDown);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
