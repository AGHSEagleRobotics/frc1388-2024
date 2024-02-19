// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.vision.Limelight;

public class AutoAngleShooter extends Command {
  private final ShooterAngleSubsystem m_shooterAngleSubsystem;
  private final Limelight m_limelight;
  /** Creates a new AutoAngleShooter. */
  public AutoAngleShooter(ShooterAngleSubsystem shooterAngleSubsystem, Limelight limelight) {
    m_shooterAngleSubsystem = shooterAngleSubsystem;
    m_limelight = limelight;
    addRequirements(m_shooterAngleSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // replace this with m_limelight.getVerticalDegrees find the bug with it idk
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
      SmartDashboard.putNumber("ty", ty);
      
      double pAngle;
      double pEx;
      double goToAngle;
      pAngle = m_limelight.getDistance() * Math.E;
      pAngle -= FieldConstants.SUBLIFER_LENGTH;
      pEx = LimelightConstants.MAX_TY_VALUE / ty;
    
    goToAngle = Math.pow(pAngle, pEx);
    SmartDashboard.putNumber("Angle for linear actuator", goToAngle);

    m_shooterAngleSubsystem.setPosition(goToAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
