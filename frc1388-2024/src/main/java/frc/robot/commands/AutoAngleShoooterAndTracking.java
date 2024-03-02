// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterAngleSubsystemConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.vision.Limelight;

public class AutoAngleShoooterAndTracking extends Command {
  private final DriveTrainSubsystem m_driveTrain;
  private final ShooterAngleSubsystem m_shooterAngleSubsystem;
  private final Limelight m_limelight;
  private final PIDController m_rotationPIDController = new PIDController(LimelightConstants.TURN_P_VALUE_AUTO_TRACKING, 0, LimelightConstants.TURN_D_VALUE_AUTO_TRACKING);
  /** Creates a new AutoTracking. */
  /** Creates a new AutoAngleShooter. */
  public AutoAngleShoooterAndTracking(ShooterAngleSubsystem shooterAngleSubsystem, Limelight limelight, DriveTrainSubsystem driveTrain) {
    m_driveTrain = driveTrain;
    m_shooterAngleSubsystem = shooterAngleSubsystem;
    m_limelight = limelight;
    addRequirements(m_driveTrain, m_shooterAngleSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_rotationPIDController.setTolerance(AutoConstants.TURN_P_TOLERANCE);
    m_rotationPIDController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double omega = m_rotationPIDController.calculate(m_limelight.getAngleFromSpeaker(), 0);

      double goToAngle = m_shooterAngleSubsystem.getCurrentPosition();

      if (m_limelight.getDistance() < LimelightConstants.DISTANCE_FROM_APRILTAG_PODIUM) {
      goToAngle = LimelightConstants.SLOPE_MATH_SUBLIFER_TO_PODIUM * m_limelight.getDistance() + LimelightConstants.SHOOTER_OFFSET_SUB;
       goToAngle = MathUtil.clamp(goToAngle, ShooterAngleSubsystemConstants.kShooterPositionDown, ShooterAngleSubsystemConstants.kShooterPositionUp);
      }
      else if (m_limelight.getDistance() > LimelightConstants.DISTANCE_FROM_APRILTAG_PODIUM && m_limelight.getDistance() < LimelightConstants.DISTANCE_FROM_APRILTAG_WING) {
        goToAngle = LimelightConstants.SLOPE_MATH_PODIUM_TO_WING * m_limelight.getDistance() + LimelightConstants.SHOOTER_OFFSET_WING;
        goToAngle = MathUtil.clamp(goToAngle, ShooterAngleSubsystemConstants.kShooterPositionWing, ShooterAngleSubsystemConstants.kShooterPositionDown);
      }

    if (m_limelight.getAprilTagID() == 4 || m_limelight.getAprilTagID() == 7) {
    m_shooterAngleSubsystem.setPosition(goToAngle);
    m_driveTrain.drive(0, 0, omega);
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
