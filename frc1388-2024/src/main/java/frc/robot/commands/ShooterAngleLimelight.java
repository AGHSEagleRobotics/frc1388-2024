// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterAngleSubsystemConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.vision.Limelight;

public class ShooterAngleLimelight extends Command {
  private final ShooterAngleSubsystem m_shooterAngleSubsystem;
  private final Limelight m_limelight;
  /** Creates a new AutoAngleShooter. */
  public ShooterAngleLimelight(ShooterAngleSubsystem shooterAngleSubsystem, Limelight limelight) {
    m_shooterAngleSubsystem = shooterAngleSubsystem;
    m_limelight = limelight;
    addRequirements(m_shooterAngleSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double goToAngle = m_shooterAngleSubsystem.getCurrentPosition();

    double distance = m_limelight.getDistance();
    double distance2 = distance * distance;

    // mycurvefit numbers for quadratic interpolation
    if ((distance > 0) && (distance < 4)) {
    goToAngle = LimelightConstants.QUADRATIC_AUTO_SHOOTER_A +
                (LimelightConstants.QUADRATIC_AUTO_SHOOTER_B * distance) +
                (LimelightConstants.QUADRATIC_AUTO_SHOOTER_C * distance2);
    }
  
    if (m_limelight.getAprilTagID() == 4 || m_limelight.getAprilTagID() == 7) {
    m_shooterAngleSubsystem.setPosition(goToAngle);
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
