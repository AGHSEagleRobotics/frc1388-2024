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

    if (m_limelight.getDistance() < LimelightConstants.DISTANCE_FROM_APRILTAG_POSITIONB) {
      goToAngle = LimelightConstants.SLOPE_MATH_SUBLIFER_TO_POSITIONB * m_limelight.getDistance() + LimelightConstants.SHOOTER_OFFSET_SUBTOB;
       goToAngle = MathUtil.clamp(goToAngle, ShooterAngleSubsystemConstants.kShooterPositionNoteB, ShooterAngleSubsystemConstants.kShooterPositionUp);
      }
      else if (m_limelight.getDistance() > LimelightConstants.DISTANCE_FROM_APRILTAG_POSITIONB && m_limelight.getDistance() < LimelightConstants.DISTANCE_FROM_APRILTAG_PODIUM) {
        goToAngle = LimelightConstants.SLOPE_MATH_POSITIONB_TO_PODIUM * m_limelight.getDistance() + LimelightConstants.SHOOTER_OFFSET_B_TO_POD;
      }
      else if (m_limelight.getDistance() > LimelightConstants.DISTANCE_FROM_APRILTAG_PODIUM && m_limelight.getDistance() < LimelightConstants.DISTANCE_FROM_APRILTAG_WING) {
        goToAngle = LimelightConstants.SLOPE_MATH_PODIUM_TO_WING * m_limelight.getDistance() + LimelightConstants.SHOOTER_OFFSET_WING;
        goToAngle = MathUtil.clamp(goToAngle, ShooterAngleSubsystemConstants.kShooterPositionWing, ShooterAngleSubsystemConstants.kShooterPositionDown);
      }
  
    if (m_limelight.getAprilTagID() == 4 || m_limelight.getAprilTagID() == 7) {
    m_shooterAngleSubsystem.setPosition(goToAngle);
    }
    else {
      m_shooterAngleSubsystem.setPosition(m_shooterAngleSubsystem.getCurrentPosition());
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
