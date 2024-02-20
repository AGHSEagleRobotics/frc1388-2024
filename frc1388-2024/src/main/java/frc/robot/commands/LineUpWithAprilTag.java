// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.LongPredicate;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.vision.Limelight;

public class LineUpWithAprilTag extends Command {

  private final DriveTrainSubsystem m_driveTrain;
  private final Limelight m_limelight;
  
  private double m_tyOffset;
  private double m_rotationGoal;
  private Debouncer m_canSeePieceDebouncer;

  private final PIDController m_rotationPIDController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);
  private final PIDController m_xPIDController = new PIDController(0.05, 0, 0);
  private final PIDController m_yPIDController = new PIDController(0.2, 0, 0);
  /** Creates a new LineUpWithAprilTag. */
  public LineUpWithAprilTag(DriveTrainSubsystem driveTrain, Limelight limelight, double tyOffset, double rotationGoal) {
    m_driveTrain = driveTrain;
    m_limelight = limelight;
    m_tyOffset = tyOffset;
    m_rotationGoal = rotationGoal;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_xPIDController.setSetpoint(0);
    m_xPIDController.reset();

    m_yPIDController.setSetpoint(m_tyOffset);
    m_yPIDController.reset();

    m_rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_rotationPIDController.setSetpoint(m_rotationGoal);
    m_rotationPIDController.reset();

    m_canSeePieceDebouncer = new Debouncer(0.1, DebounceType.kFalling);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  if (!m_canSeePieceDebouncer.calculate(m_limelight.getIsTargetFound())) {
      m_driveTrain.drive(0, 0, 0);
      return;
    }

    // replace this with m_limelight.getVerticalDegrees find the bug with it idk
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    double tx = m_limelight.getdegRotationToTarget();
    double rotationError = m_driveTrain.getPose().getRotation().getRadians();

    double txTolerance = 0.5;
    if (isWithinTolerance(ty, m_tyOffset, 4)) {
      txTolerance = Math.copySign(txTolerance * 2, txTolerance);
    }

    if (isWithinTolerance(rotationError, 0, 0.25)) {
      rotationError = 0;
    }

    // Reduce tx based on how far off our rotation is so the x controller doesn't
    // over compensate
    tx -= Rotation2d.fromRadians(rotationError).getDegrees();
    if (isWithinTolerance(tx, 0, txTolerance)) {
      tx = 0;
    }
    if (isWithinTolerance(ty, m_tyOffset, 0.5)) {
      ty = m_tyOffset;
    }
    double xVelocity = -m_yPIDController.calculate(ty);
    double yVelocity = -m_xPIDController.calculate(tx);
    double omega = m_rotationPIDController.calculate(rotationError);
    m_driveTrain.driveRobotRelative(ChassisSpeeds.fromRobotRelativeSpeeds(xVelocity, yVelocity, omega, new Rotation2d()));


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }


  public static boolean isWithinTolerance(
      double currentValue, double targetValue, double tolerance) {
    return Math.abs(currentValue - targetValue) <= tolerance;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
