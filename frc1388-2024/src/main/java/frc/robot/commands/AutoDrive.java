// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoDrive extends Command {

  private final DriveTrainSubsystem m_driveTrain;
  private final double m_setPoint;
  private double m_start;

  private final PIDController m_driveController = new PIDController(0.06, 0.015, 0);
  private final SlewRateLimiter m_accelerationLimiter = new SlewRateLimiter(0.4);

  /** Creates a new AutoDrive. */
  public AutoDrive(double setpoint, DriveTrainSubsystem driveTrainSubsystem) {
    m_driveTrain = driveTrainSubsystem;
    m_setPoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_start = m_driveTrain.getDistTraveled();
    m_accelerationLimiter.calculate(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double pidInput = -m_setPoint  + (m_driveTrain.getDistTraveled() - m_start);
    // SmartDashboard.putNumber("drive pid input", pidInput);
    // SmartDashboard.putString("auto drive info", "setpoint" + m_setPoint + "dt.getdist" + m_driveTrain.getDistTraveled() + "start" + m_start);
    double pidOutput = m_driveController.calculate(m_driveTrain.getDistTraveled() - m_start, m_setPoint);
    double slewOutput = m_accelerationLimiter.calculate(pidOutput);
    SmartDashboard.putNumber("AutoDrive/drive slew", slewOutput);
    m_driveTrain.differentialDrive(slewOutput);
    SmartDashboard.putNumber("AutoDrive/dist traveled", m_driveTrain.getDistTraveled() - m_start);
    SmartDashboard.putNumber("AutoDrive/total dist traveled", m_driveTrain.getDistTraveled());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_driveTrain.getDistTraveled() - m_start) > Math.abs(m_setPoint) - 0.01;
  }
}
