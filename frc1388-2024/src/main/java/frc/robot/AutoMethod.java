package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import javax.sound.midi.ShortMessage;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
public class AutoMethod {

  /** Creates a new AutoMethod. */

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final Dashboard m_dashboard;
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intakeSubsystem;
  private final TransitionSubsystem m_transitionSubsystem;

  public AutoMethod(DriveTrainSubsystem driveTrainSubsystem, Dashboard dashboard, ShooterSubsystem shooter, IntakeSubsystem intake, TransitionSubsystem transition) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_dashboard = dashboard;
    m_shooter = shooter;
    m_intakeSubsystem = intake;
    m_transitionSubsystem = transition;
  }

  public Command SitStillLookPretty(){
    return new AutoDrive(0, m_driveTrainSubsystem);
  }

  public Command Start1Leave(){
    return new AutoDrive(AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem);
  }

  public Command Shoot1IntakeBSpeakerB(){
    return new ShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter).withTimeout(2)
    .alongWith(
      new FeedShooter(m_transitionSubsystem, m_intakeSubsystem).withTimeout(2)
    )
    .alongWith(
     new WaitCommand(1.0)
    )
    .andThen(
      new DeployIntakeCommand(m_intakeSubsystem)
    )
    .andThen(
      new AutoDrive(AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem)
    )
    .alongWith(
      new WaitCommand(1.0)
    )
    .andThen(
      new RetractIntakeCommand(m_intakeSubsystem)
    )
    .alongWith(
      new AutoDrive(-AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem)
    )
    .alongWith(
      new WaitCommand(1.0)
    )
    .andThen(
      new ShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter).withTimeout(2)
    )
    .alongWith(
      new FeedShooter(m_transitionSubsystem, m_intakeSubsystem).withTimeout(2)
    );
    
  }
  
  public Command ShootAndLeave(){
    return new ShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter).withTimeout(2)
    .alongWith(
      new FeedShooter(m_transitionSubsystem, m_intakeSubsystem).withTimeout(2)
    )
    .alongWith(
      new WaitCommand(1)
    )
    .andThen(
      new AutoDrive(AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem)
    );
  }

  public Command getAutonomousCommand() {
        AutoConstants.Objective objective = m_dashboard.getObjective();
        DataLogManager.log("####### objective:" + objective);
    
        if (objective == null) {
          return null;
        }
    
        switch (objective) {
    
          case SITSTILL:
            return SitStillLookPretty();
    
          case START1LEAVE:
            return Start1Leave();

          case LEAVEANDSHOOT:
            return ShootAndLeave();

          case Shoot1IntakeBSpeakerB:
            return Shoot1IntakeBSpeakerB();
        }
        return null;
      }
}