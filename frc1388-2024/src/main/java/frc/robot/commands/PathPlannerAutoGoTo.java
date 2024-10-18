// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainSubsystem;

public class PathPlannerAutoGoTo extends Command {
  private final DriveTrainSubsystem m_driveTrain;
private Pose2d m_setpoint;
  /** Creates a new PathPlannerAutoGoTo. */
  public PathPlannerAutoGoTo(Pose2d setpoint, DriveTrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = drivetrain;

    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PathConstraints constraints = new PathConstraints(
        4.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    AutoBuilder.pathfindToPose(m_setpoint, constraints);
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
