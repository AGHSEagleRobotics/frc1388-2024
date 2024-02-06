// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.vision.Limelight;

public class ConstantTurnToAprilTag extends Command {
  private final DriveTrainSubsystem m_driveTrain;
  private final Limelight m_limelight;
  private final double m_turnSpeed;
  private final AHRS m_navxGyro;
   private final PIDController m_pidController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);
  /** Creates a new ConstantTurnToAprilTag. */
  public ConstantTurnToAprilTag(DriveTrainSubsystem driveTrain, Limelight limelight, double turnSpeed, AHRS navxGyro) {
    m_driveTrain = driveTrain; 
    m_limelight = limelight;
    m_turnSpeed = turnSpeed;
    m_navxGyro = navxGyro;
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
    double speed;
    double turnMinSpeed;
    double angle = m_navxGyro.getAngle();
    double rate = m_navxGyro.getRate();

    if (rate > AutoConstants.TURN_MIN_SPEED_THRESHOLD) {
      turnMinSpeed = AutoConstants.TURN_MIN_SPEED_MOVING;
    }
    else {
      turnMinSpeed = AutoConstants.TURN_MIN_SPEED_STOPPED;
    }

    speed = m_pidController.calculate(m_limelight.getAngleFromSpeaker(), angle);
    if (speed > 0) {
      speed = MathUtil.clamp(speed, turnMinSpeed, m_turnSpeed);
    } else {
      speed = MathUtil.clamp(speed, -m_turnSpeed, -turnMinSpeed);
    }

    if (m_limelight.getAprilTagID() == 4) {
      if (m_limelight.getdegRotationToTarget() >= 2) {
        new ConstantTurnToAprilTag(m_driveTrain, m_limelight, turnMinSpeed, m_navxGyro);
      }

      if (m_limelight.getdegRotationToTarget() <= 2) {
        new ConstantTurnToAprilTag(m_driveTrain, m_limelight, turnMinSpeed, m_navxGyro);
      } else if (m_limelight.getdegRotationToTarget() >= 2 && m_limelight.getdegRotationToTarget() <= 2) {
        m_driveTrain.drive(0, 0, 0);
      }
    }
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
