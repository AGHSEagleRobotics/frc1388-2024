package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Intake;
public class AutoMethod {

  /** Creates a new AutoMethod. */

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private Dashboard m_dashboard;
  private Intake m_intake;

  public AutoMethod(DriveTrainSubsystem driveTrainSubsystem, Dashboard dashboard, Intake intake) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_dashboard = dashboard;
    m_intake = intake;
  }

  public Command SitStillLookPretty(){
    return new AutoDrive(m_driveTrainSubsystem, 0);
  }

  public Command MoveOutOfZone(){
    return new AutoDrive(m_driveTrainSubsystem, 2);
  }
}