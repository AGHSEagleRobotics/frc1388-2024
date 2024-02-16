// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.vision.Limelight;

public class AutoTracking extends Command {
  private final DriveTrainSubsystem m_driveTrain;
  private final Limelight m_limelight;

  private final PIDController m_limelightPIDController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);
  private PIDController m_rotationController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);
  /** Creates a new AutoTracking. */
  public AutoTracking(DriveTrainSubsystem driveTrain, Limelight limelight) {
    m_driveTrain = driveTrain;
    m_limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelightPIDController.setTolerance(AutoConstants.TURN_P_TOLERANCE);
    m_limelightPIDController.enableContinuousInput(0, 360);

    m_rotationController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double omega = m_limelightPIDController.calculate(m_limelight.getAngleFromSpeaker());
    m_driveTrain.drive(0, 0, omega);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
