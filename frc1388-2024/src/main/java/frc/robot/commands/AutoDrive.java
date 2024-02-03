// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.xml.stream.events.StartDocument;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoDrive extends Command {

  private final DriveTrainSubsystem m_driveTrainSubsystem;
  private double m_start;
  private final double m_totalDist;

  SlewRateLimiter filter = new SlewRateLimiter(0.1);
  PIDController pid = new PIDController(0.1, 0, 0);

  /** Creates a new AutoDrive. */
  public AutoDrive(DriveTrainSubsystem driveTrainSubsystem, double distance) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_totalDist = distance;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_start = m_driveTrainSubsystem.getDistTraveled();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double pidInput = 2 * m_totalDist - Math.abs(m_start - m_driveTrainSubsystem.getDistTraveled());
    // SmartDashboard.putNumber("pid input", pidInput);
    // double pidOutput = pid.calculate(pidInput);
    // SmartDashboard.putNumber("Pid output", pidOutput);

    // m_driveTrainSubsystem.differentialDrive(filter.calculate(0.1) - pidOutput, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrainSubsystem.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // SmartDashboard.putNumber("pid input", (Math.abs(m_start - m_driveTrainSubsystem.getDistTraveled())));
    // SmartDashboard.putNumber("dist", Math.abs(m_start - m_driveTrainSubsystem.getDistTraveled()));
    // SmartDashboard.putNumber("m_start", m_start);
    // SmartDashboard.putNumber("dt dist traveled", m_driveTrainSubsystem.getDistTraveled());
    // return Math.abs(m_start - m_driveTrainSubsystem.getDistTraveled()) > m_totalDist;
  }
}
