package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Intake;
public class AutoMethod {

  /** Creates a new AutoMethod. */

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final Dashboard m_dashboard;

  public AutoMethod(DriveTrainSubsystem driveTrainSubsystem, Dashboard dashboard) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_dashboard = dashboard;
  }

  public Command SitStillLookPretty(){
    return new AutoDrive(m_driveTrainSubsystem, 0);
  }

  public Command MoveOutOfZone(){
    return new AutoDrive(m_driveTrainSubsystem, 2);
    // distance needs to be changed to a Constant
    // need to create multiple methods depending on where you start
  }

  public Command getAutonomousCommand() {
        AutoConstants.Objective objective = m_dashboard.getObjective();
        AutoConstants.Position position = m_dashboard.getPosition();
        DataLogManager.log("####### objective:" + objective);
        DataLogManager.log("####### position:" + position);
    
        if (objective == null || position == null) {
          return null;
        }
    
        switch (objective) {
    
          case SITSTILL:
            return SitStillLookPretty();
    
          case LEAVEZONE:
            return MoveOutOfZone();
        }
        return null;
      }
}