// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

/** this command moves the robot to an x, y location */
public class AutoGoToPoint extends Command {

  private final DriveTrainSubsystem m_driveTrain;

  private final double X_SETPOINT;
  private final double Y_SETPOINT;

  // i was 0.015
  private final PIDController m_xController = new PIDController(0.1, 0, 0);
  private double m_lastXSpeed = 0;
  private final SlewRateLimiter m_xAccLimiter = new SlewRateLimiter(0.3);

  private final PIDController m_yController = new PIDController(0.1, 0, 0);
  private double m_lastYSpeed = 0;
  private final SlewRateLimiter m_yAccLimiter = new SlewRateLimiter(0.3);

  /** Creates a new AutoMove. */
  public AutoGoToPoint(double xSetpoint, double ySetpoint, DriveTrainSubsystem drivetrain) {

    m_driveTrain =  drivetrain;

    X_SETPOINT = xSetpoint;
    Y_SETPOINT = ySetpoint;

    m_xController.setTolerance(0.03);
    m_yController.setTolerance(0.03);
    
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetPose(new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = m_xController.calculate(m_driveTrain.getPose().getX(), X_SETPOINT);
    if (xSpeed > m_lastXSpeed) {
      xSpeed = m_xAccLimiter.calculate(xSpeed);
    }

    double ySpeed = m_yController.calculate(m_driveTrain.getPose().getY(), Y_SETPOINT);
    if (ySpeed > m_lastYSpeed) {
      ySpeed = m_yAccLimiter.calculate(ySpeed);
    }

    m_driveTrain.drive(xSpeed, ySpeed, 0);
    m_lastXSpeed = xSpeed;
    m_lastYSpeed = ySpeed;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_xController.atSetpoint() && m_yController.atSetpoint();
    // return false;
  }
}
