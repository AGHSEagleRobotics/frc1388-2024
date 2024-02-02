// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoDrive extends Command {

  private final DriveTrainSubsystem m_driveTrain;
  private final double m_setPoint;
  private double m_start;

  private final PIDController m_driveController = new PIDController(0.1, 0, 0);

  /** Creates a new AutoDrive. */
  public AutoDrive(DriveTrainSubsystem driveTrainSubsystem, double setpoint) {
    m_driveTrain = driveTrainSubsystem;
    m_setPoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_start = m_driveTrain.getDistTraveled();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidInput = m_driveTrain.getDistTraveled() - m_start - m_setPoint;
    SmartDashboard.putNumber("drive pid input", pidInput);
    double pidOutput = m_driveController.calculate(pidInput);
    SmartDashboard.putNumber("drive pid output", pidOutput);
    m_driveTrain.differentialDrive(pidOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.getDistTraveled() - m_start > m_setPoint;
  }
}
