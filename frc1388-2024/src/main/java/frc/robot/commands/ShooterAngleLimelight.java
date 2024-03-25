// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;

public class ShooterAngleLimelight extends Command {
  private final ShooterAngleSubsystem m_shooterAngleSubsystem;
  private final DriveTrainSubsystem m_driveTrainSubsystem;
  
  /** Creates a new AutoAngleShooter. */
  public ShooterAngleLimelight(ShooterAngleSubsystem shooterAngleSubsystem, DriveTrainSubsystem driveTrainSubsystem) {
    m_shooterAngleSubsystem = shooterAngleSubsystem;
    m_driveTrainSubsystem = driveTrainSubsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterAngleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double goToAngle = m_shooterAngleSubsystem.getCurrentPosition();

    double distance = m_driveTrainSubsystem.getAbsouluteDistanceFromSpeaker();

    double distance2 = distance * distance;

    // mycurvefit numbers for quadratic interpolation
    if ((distance > 0) && (distance < LimelightConstants.DISTANCE_FROM_APRILTAG_AUTOSHOOTER)) {
      goToAngle = LimelightConstants.QUADRATIC_AUTO_SHOOTER_A +
          (LimelightConstants.QUADRATIC_AUTO_SHOOTER_B * distance) +
          (LimelightConstants.QUADRATIC_AUTO_SHOOTER_C * distance2);
    }
    m_shooterAngleSubsystem.setPosition(goToAngle);
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
