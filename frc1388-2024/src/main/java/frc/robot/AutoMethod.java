package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterAngleSubsystemConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShooterAngleLimelight;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoGoToPoint;
import frc.robot.commands.AutoShooterAngle;
import frc.robot.commands.AutoShooterCommand;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.IntakeTransitionCommand;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.GoToNote;
import frc.robot.commands.LineUpWithAprilTag;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeTransitionCommand.IntakeTransState;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.vision.Limelight;
public class AutoMethod {

  /** Creates a new AutoMethod. */

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private final Dashboard m_dashboard;
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intakeSubsystem;
  private final TransitionSubsystem m_transitionSubsystem;
  private final ShooterAngleSubsystem m_shooterAngleSubsystem;
  private final Limelight m_limelight;

 
  public AutoMethod(DriveTrainSubsystem driveTrainSubsystem, Dashboard dashboard, ShooterSubsystem shooter, IntakeSubsystem intake, TransitionSubsystem transition, ShooterAngleSubsystem shooterAngle, Limelight limelight) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_dashboard = dashboard;
    m_shooter = shooter;
    m_intakeSubsystem = intake;
    m_transitionSubsystem = transition;
    m_limelight = limelight;
    m_shooterAngleSubsystem = shooterAngle;
  }

  public Command SitStillLookPretty(){
    return new AutoDrive(0, m_driveTrainSubsystem);
  }

  public Command Start1Leave(){
    return new AutoDrive(AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem);
  }

  public Command Start123Shoot(){
    return new SequentialCommandGroup(
      new AutoShooterAngle(ShooterAngleSubsystemConstants.kShooterPositionSpeaker, m_shooterAngleSubsystem),
      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
        .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem))
    );
  }
  public Command ShootAndLeave(){
    return new SequentialCommandGroup(
      new AutoShooterAngle(ShooterAngleSubsystemConstants.kShooterPositionSpeaker, m_shooterAngleSubsystem),
      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
        .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)),
        new AutoDrive(AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem));
  }

  public Command ShootAndLeaveFromSideWithLonger(){
    return new SequentialCommandGroup(
      new AutoShooterAngle(ShooterAngleSubsystemConstants.kShooterPositionSpeaker, m_shooterAngleSubsystem),
      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
        .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)),
        new AutoDrive(AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem));
  }

  // needs testing
  public Command Shoot3Leave(){
    return new SequentialCommandGroup(
     new AutoShooterAngle(ShooterAngleSubsystemConstants.kShooterPositionSpeaker, m_shooterAngleSubsystem),
     new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
       .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)),
      new AutoGoToPoint(2, -1.27, 135, m_driveTrainSubsystem)
    );
  }
  
  // needs testing
  public Command Shoot2Leave(){
    return new SequentialCommandGroup(
     new AutoShooterAngle(ShooterAngleSubsystemConstants.kShooterPositionSpeaker, m_shooterAngleSubsystem),
     new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
       .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)),
      new AutoGoToPoint(2, -1.27, 135, m_driveTrainSubsystem)
    );
  }

  public Command Shoot1IntakeBSpeakerB(){
    return new SequentialCommandGroup(
      new AutoShooterAngle(ShooterAngleSubsystemConstants.kShooterPositionSpeaker, m_shooterAngleSubsystem),
      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
        .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)),
      new AutoShooterAngle(ShooterAngleSubsystemConstants.kShooterPositionNoteB, m_shooterAngleSubsystem)
        .alongWith( new IntakeTransitionCommand(IntakeTransState.DEPLOYING, false, m_intakeSubsystem, m_transitionSubsystem, m_limelight)
          .deadlineWith(new WaitCommand(0.25)
            .andThen(new AutoDrive(AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem)))), 
      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
        .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)) 
    );
  }
  
  public Command Shoot1IntakeBSpeakerBIntakeASpeakerA(){
    return new ShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter).withTimeout(2.0)
    .alongWith(
      new FeedShooter(m_transitionSubsystem, m_intakeSubsystem).withTimeout(2)
    )
    .alongWith(
     new WaitCommand(1.0)
    )
    .andThen(
      new DeployIntakeCommand(m_intakeSubsystem, m_transitionSubsystem)
    )
    .andThen(
      new AutoDrive(AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem)
    )
    .alongWith(
      new WaitCommand(1.0)
    )
    .andThen(
      new RetractIntakeCommand(m_intakeSubsystem, m_transitionSubsystem)  // FIXME: use IntakeTransitionCommand
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
    )
    .alongWith(
      new WaitCommand(1.0)
    )
    .andThen(
      new AutoGoToPoint(-1.75,0, 180, m_driveTrainSubsystem)
    )
    .alongWith(
      new WaitCommand(1.0)
    )
    .andThen(
      new DeployIntakeCommand(m_intakeSubsystem, m_transitionSubsystem)
    )
    .andThen(
      new AutoGoToPoint(0,-2, 180, m_driveTrainSubsystem)
    )
    .alongWith(
      new WaitCommand(1.0)
    )
    .andThen(
      new RetractIntakeCommand(m_intakeSubsystem, m_transitionSubsystem)  // FIXME: use IntakeTransitionCommand
    )
    .andThen(
      new AutoGoToPoint(1.75, 0, 180, m_driveTrainSubsystem)    
    )
    .andThen(
      new AutoDrive(-AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem)
    )
    .alongWith(
      new WaitCommand(1.0)
    )
    .andThen(
      new ShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter)
    )
    .alongWith(
      new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)
    );
  }

  public Command testCoordinate(){
     return new SequentialCommandGroup(
      new IntakeTransitionCommand(IntakeTransState.DEPLOYING, false, m_intakeSubsystem, m_transitionSubsystem, m_limelight)
          .deadlineWith(new WaitCommand(0.25),
          new GoToNote(m_driveTrainSubsystem, m_limelight, m_intakeSubsystem)));
  }

  public Command Shoot1IntakeBSpeakerBIntakeCSpeakerC(){
    return new ShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter).withTimeout(2.0)
    .alongWith(
      new FeedShooter(m_transitionSubsystem, m_intakeSubsystem).withTimeout(2)
    )
    .alongWith(
     new WaitCommand(1.0)
    )
    .andThen(
      new DeployIntakeCommand(m_intakeSubsystem, m_transitionSubsystem)
    )
    .andThen(
      new AutoDrive(AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem)
    )
    .alongWith(
      new WaitCommand(1.0)
    )
    .andThen(
      new RetractIntakeCommand(m_intakeSubsystem, m_transitionSubsystem)  // FIXME: use IntakeTransitionCommand
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
  
  public Command LimelightShoot1IntakeBSpeakerB() {
    return new ShooterAngleLimelight(m_shooterAngleSubsystem, m_limelight)
        .alongWith(
          new SequentialCommandGroup(
            new ShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter).withTimeout(2.0)
                .alongWith(
                    new FeedShooter(m_transitionSubsystem, m_intakeSubsystem).withTimeout(2.0)),
            new GoToNote(m_driveTrainSubsystem, m_limelight, m_intakeSubsystem)
                .deadlineWith(new DeployIntakeCommand(m_intakeSubsystem, m_transitionSubsystem)),
            new ShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter)
                .alongWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem))));
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

          case Shoot123:
            return Start123Shoot();

          case ShootAndLeaveFromSideWithLonger:
            return ShootAndLeaveFromSideWithLonger();

          case testCoordinate:
            return testCoordinate();

          // case Shoot3Leave:
          //   return Shoot3Leave();

          // case Shoot1IntakeBSpeakerBIntakeASpeakerA:
          //   return Shoot1IntakeBSpeakerB();

        }
        return null;
      }
}