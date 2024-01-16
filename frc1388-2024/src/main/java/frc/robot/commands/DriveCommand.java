// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveCommand extends Command {

  private final DriveTrainSubsystem m_driveTrain;

  private final Supplier<Double> m_leftY;
  private final Supplier<Double> m_leftX;
  private final Supplier<Double> m_rightX;


  /** Creates a new DriveCommand. */
  public DriveCommand(DriveTrainSubsystem driveTrain, Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX) {
    m_driveTrain = driveTrain;

    m_leftY = leftY;
    m_leftX = leftX;
    m_rightX = rightX;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = -3.0 * scale(MathUtil.applyDeadband(m_leftY.get(), 0.1), 2.5);
    double yVelocity = -3.0 * scale(MathUtil.applyDeadband(m_leftX.get(), 0.1), 2.5);
    double omega = 2 * Math.PI * -scale(MathUtil.applyDeadband(m_rightX.get(), 0.1), 5);
    //lots of magic numbers check what the names should be
    m_driveTrain.drive(-xVelocity, -yVelocity, -omega); // max speed: 3 m/s transitional, pi rad/s (0.5 rotation/s) rotational (for now)
  }

  /**this method needs documentation */
  private double scale(double in, double scale) {
    return Math.tan(in * Math.atan(scale)) / scale;
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
