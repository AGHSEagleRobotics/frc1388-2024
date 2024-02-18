// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.vision.Limelight;

public class GoToNote extends Command {
    private final DriveTrainSubsystem m_driveTrain;
    private final Limelight m_limelight;
    private final IntakeSubsystem m_intakeSubsystem;
    private Pose2d m_initialPose;
    private Debouncer m_canSeePieceDebouncer;

    private final PIDController m_driveController = new PIDController(0.03, 0.015, 0);
    private final PIDController m_limelightPIDController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);
  /** Creates a new GoToNote. */
  public GoToNote(DriveTrainSubsystem driveTrainSubsystem, Limelight limelight, IntakeSubsystem intakeSubsystem) {
    m_driveTrain = driveTrainSubsystem;
    m_limelight = limelight;
    m_intakeSubsystem = intakeSubsystem;
    m_initialPose = m_driveTrain.getPose();
    addRequirements(m_driveTrain, m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelightPIDController.setTolerance(AutoConstants.TURN_P_TOLERANCE);
    m_limelightPIDController.enableContinuousInput(0, 360);
    m_canSeePieceDebouncer = new Debouncer(0.1, DebounceType.kFalling);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_canSeePieceDebouncer.calculate(m_limelight.getIsTargetFound())) {
      m_driveTrain.drive(0, 0, 0);
      return;
    }

    double omega = m_limelightPIDController.calculate(m_limelight.getAngleFromSpeaker(), 0);
    double xVelocity = distanceTraveled() > 1 ? 2 / 2 : 2;
    m_driveTrain.drive(xVelocity, 0, omega);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }

  private double distanceTraveled() {
    return Math.abs(m_driveTrain.getPose().getTranslation().getDistance(m_initialPose.getTranslation()));
  }

  private boolean isDistanceTooFar() {
    return Math.abs(distanceTraveled()) > 3;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intakeSubsystem.isNoteDetected() || isDistanceTooFar();
  }
}
