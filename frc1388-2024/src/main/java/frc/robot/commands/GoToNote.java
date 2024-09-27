// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.vision.Limelight;

public class GoToNote extends Command {
  private final DriveTrainSubsystem m_driveTrain;
  private final Limelight m_limelight;
  private final IntakeSubsystem m_intakeSubsystem;
  private Pose2d m_initialPose;
  private Debouncer m_canSeePieceDebouncer;

  private final PIDController m_xController = new PIDController(1.8, 0, 0);
  private double m_lastXSpeed = 0;
  private final SlewRateLimiter m_xAccLimiter = new SlewRateLimiter(0.2);

  private final PIDController m_yController = new PIDController(1.8, 0, 0);
  private double m_lastYSpeed = 0;
  private final SlewRateLimiter m_yAccLimiter = new SlewRateLimiter(0.2);

  private final PIDController m_turnPidController = new PIDController(LimelightConstants.TURN_P_VALUE_AUTO_TRACKING, 0, LimelightConstants.TURN_D_VALUE_AUTO_TRACKING);

  /** Creates a new GoToNote. */
  public GoToNote(DriveTrainSubsystem driveTrainSubsystem, Limelight limelight, IntakeSubsystem intakeSubsystem) {
    m_driveTrain = driveTrainSubsystem;
    m_limelight = limelight;
    m_intakeSubsystem = intakeSubsystem;
    m_initialPose = m_driveTrain.getPose();
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turnPidController.setTolerance(AutoConstants.TURN_P_TOLERANCE);
    m_turnPidController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // if (!m_canSeePieceDebouncer.calculate(m_limelight.getIsNoteFound())) {
    //   m_driveTrain.drive(0, 0, 0);
    //   return;
    // }
    
    double omega = m_driveTrain.getTurnToNoteSpeed(m_turnPidController);
    double xSpeed = m_xController.calculate(m_driveTrain.getPose().getX(), m_driveTrain.getNotePose().getX());
    double ySpeed = m_xController.calculate(m_driveTrain.getPose().getY(), m_driveTrain.getNotePose().getY());
    if (m_limelight.getIsNoteFound()) {
      m_driveTrain.drive(xSpeed, ySpeed, omega);
    } else {
      m_driveTrain.drive(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
    m_turnPidController.setTolerance(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.isNoteDetected();
  }
}
