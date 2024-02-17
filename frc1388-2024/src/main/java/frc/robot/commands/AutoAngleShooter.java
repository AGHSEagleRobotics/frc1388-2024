// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.vision.Limelight;

public class AutoAngleShooter extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final Limelight m_limelight;
  /** Creates a new AutoAngleShooter. */
  public AutoAngleShooter(ShooterSubsystem shooterSubsystem, Limelight limelight) {
    m_shooterSubsystem = shooterSubsystem;
    m_limelight = limelight;
    addRequirements(m_shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pAngle;
    double pEx;
    double goToAngle;
    pAngle = m_limelight.getDistance() * Math.E;
    pAngle -= FieldConstants.SUBLIFER_LENGTH;
    pEx = m_limelight.getdegVerticalToTarget() / LimelightConstants.MAX_DISTANCE;
    
    goToAngle = Math.pow(pAngle, pEx);
    SmartDashboard.putNumber("Angle for linear actuator", goToAngle);
    // set the linear actuator to that value
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // default value from where the lienar actuator
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
