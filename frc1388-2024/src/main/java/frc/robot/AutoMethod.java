package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;

import frc.robot.commands.AutoDrive;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
public class AutoMethod {

  /** Creates a new AutoMethod. */

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final Dashboard m_dashboard;
  private final ShooterSubsystem m_shooter;

  public AutoMethod(DriveTrainSubsystem driveTrainSubsystem, Dashboard dashboard, ShooterSubsystem shooter) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_dashboard = dashboard;
    m_shooter = shooter;
  }

  public Command SitStillLookPretty(){
    return new AutoDrive(0, m_driveTrainSubsystem);
  }

  public Command MoveOutOfZone(){
    return new AutoDrive(AutoConstants.LEAVE_ZONE_DIST, m_driveTrainSubsystem);
    // distance needs to be changed to a Constant
    // need to create multiple methods depending on where you start
  }

  public Command ShootAndLeave(){
    return new ShooterCommand(m_shooter)
    .alongWith(
      new WaitCommand(1.0)
    )
    .andThen(
      new AutoDrive(AutoConstants.LEAVE_ZONE_DIST, m_driveTrainSubsystem)
    );
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

          case LEAVEANDSHOOT:
            if (position == AutoConstants.Position.MID){
              return ShootAndLeave();
            }
            else{
              return null;
            }
        }
        return null;
      }
}