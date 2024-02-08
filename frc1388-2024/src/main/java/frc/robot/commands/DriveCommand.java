// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.vision.Limelight;

public class DriveCommand extends Command {

  private final DriveTrainSubsystem m_driveTrain;
  private final Limelight m_limelight;

  private final Supplier<Double> m_leftY;
  private final Supplier<Double> m_leftX;
  private final Supplier<Double> m_rightX;
  private final Supplier<Boolean> m_start;
  private final PIDController m_pidController = new PIDController(AutoConstants.TURN_P_VALUE, AutoConstants.TURN_I_VALUE, AutoConstants.TURN_D_VALUE);

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveTrainSubsystem driveTrain, Limelight limelight, Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX, Supplier<Boolean> start) {
    m_driveTrain = driveTrain;

    m_leftY = leftY;
    m_leftX = leftX;
    m_rightX = rightX;

    m_limelight = limelight;
    m_start = start;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setTolerance(AutoConstants.TURN_P_TOLERANCE);
    m_pidController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = Constants.DriveTrainConstants.ROBOT_MAX_SPEED * -scale(MathUtil.applyDeadband(m_leftY.get(), Constants.DriveCommandConstants.CONTROLLER_DEADBAND), Constants.DriveCommandConstants.LEFT_STICK_SCALE);
    double yVelocity = Constants.DriveTrainConstants.ROBOT_MAX_SPEED * -scale(MathUtil.applyDeadband(m_leftX.get(), Constants.DriveCommandConstants.CONTROLLER_DEADBAND), Constants.DriveCommandConstants.LEFT_STICK_SCALE);
    // 

    double omega = 0;
    if (m_start.get())
    {
      omega = m_pidController.calculate(m_limelight.getAngleFromSpeaker());
    }
    else {
      omega = 2 * Math.PI * -scale(MathUtil.applyDeadband(m_rightX.get(), Constants.DriveCommandConstants.CONTROLLER_DEADBAND), Constants.DriveCommandConstants.RIGHT_STICK_SCALE);
    }
    
    m_driveTrain.drive(xVelocity, yVelocity, omega); // max speed: 3 m/s transitional, pi rad/s (0.5 rotation/s) rotational (for now)
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
