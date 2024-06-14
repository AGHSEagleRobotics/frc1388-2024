package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.commands.DriveStraight;
import frc.robot.commands.AutoFeedShooter;
import frc.robot.commands.AutoGoToPoint;
import frc.robot.commands.AutoShooterAngle;
import frc.robot.commands.AutoShooterCommand;
import frc.robot.commands.AutoTracking;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.IntakeTransitionCommand;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.GoToNote;
import frc.robot.commands.LineUpWithAprilTag;
import frc.robot.commands.PullToTransition;
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

  private final Command m_fourNote;
  private final Command m_threeNote;


 
  public AutoMethod(DriveTrainSubsystem driveTrainSubsystem, Dashboard dashboard, ShooterSubsystem shooter, IntakeSubsystem intake, TransitionSubsystem transition, ShooterAngleSubsystem shooterAngle, Limelight limelight) {
    m_driveTrainSubsystem = driveTrainSubsystem;
    m_dashboard = dashboard;
    m_shooter = shooter;
    m_intakeSubsystem = intake;
    m_transitionSubsystem = transition;
    m_limelight = limelight;
    m_shooterAngleSubsystem = shooterAngle;

    m_fourNote = FourNote();
    m_threeNote = ThreeFarNote();
  }

  public Command LearningCommands() {
    return new DriveStraight(3, m_driveTrainSubsystem).andThen(
    new AutoTurn(180, m_driveTrainSubsystem));
  }

  public Command ShootAndLeave(){
    return new SequentialCommandGroup(
      new AutoShooterAngle(ShooterAngleSubsystemConstants.kShooterPositionSpeaker, m_shooterAngleSubsystem),
      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
        .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)),
        new DriveStraight(AutoConstants.LEAVE_ZONE_FROM_SUB_DIST, m_driveTrainSubsystem));
  }

  public Command FourNote() {
    return new ShooterAngleLimelight(m_shooterAngleSubsystem)
    .alongWith(
      new SequentialCommandGroup(
      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
        .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)),

         new IntakeTransitionCommand(IntakeTransState.DEPLOYING, false, true, m_intakeSubsystem, m_transitionSubsystem, m_limelight)
          .deadlineWith(new WaitCommand(0.25)
            .andThen(makeSwerveAutoCommand("4_note.1"))), 

      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
        .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)),

      new IntakeTransitionCommand(IntakeTransState.DEPLOYING, false, true, m_intakeSubsystem, m_transitionSubsystem, m_limelight)
          .deadlineWith(new WaitCommand(0.25)
            .andThen(makeSwerveAutoCommand("4_note.2"))), 
      
      makeSwerveAutoCommand("4_note.3"),

      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
        .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)),

      
      new IntakeTransitionCommand(IntakeTransState.DEPLOYING, false, true, m_intakeSubsystem, m_transitionSubsystem, m_limelight)
          .deadlineWith(new WaitCommand(0.25)
            .andThen(makeSwerveAutoCommand("4_note.4"))),
      
      makeSwerveAutoCommand("4_note.5"),

      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
        .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem))

      
    ));
  }

  public Command ThreeFarNote() {
    return new ShooterAngleLimelight(m_shooterAngleSubsystem)
    // .alongWith(new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem))
    .alongWith(new SequentialCommandGroup(
      new WaitCommand(0.75),

      
      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
        .deadlineWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem)),
      
      makeSwerveAutoCommand("3_note.1")
        .alongWith(
          new WaitCommand(1.25)
          .andThen(new IntakeTransitionCommand(IntakeTransState.DEPLOYING, false, false, m_intakeSubsystem, m_transitionSubsystem, m_limelight))
          ),
      
      makeSwerveAutoCommand("3_note.2")
        .alongWith(
          new IntakeTransitionCommand(IntakeTransState.RETRACTING, true, true, m_intakeSubsystem, m_transitionSubsystem, m_limelight)
        ),

      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
      .deadlineWith(
        new WaitCommand(0.1)
        .andThen(new AutoTracking(m_driveTrainSubsystem, m_limelight)
                 .raceWith(new WaitCommand(0.5)))
        .andThen(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem))
        ),

      makeSwerveAutoCommand("3_note.3")
        .alongWith(
        new WaitCommand(1.5)
          .andThen(new IntakeTransitionCommand(IntakeTransState.DEPLOYING, false, false, m_intakeSubsystem, m_transitionSubsystem, m_limelight))
          ),
      
      makeSwerveAutoCommand("3_note.4")
        .alongWith(
          new IntakeTransitionCommand(IntakeTransState.RETRACTING, true, true, m_intakeSubsystem, m_transitionSubsystem, m_limelight)
        ),

      new AutoShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooter, m_transitionSubsystem)
      .deadlineWith(
        new WaitCommand(0.1)
        .andThen(new AutoTracking(m_driveTrainSubsystem, m_limelight)
                 .raceWith(new WaitCommand(0.5)))
        .andThen(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem))
        )
    ));
  }

  public Command getAutonomousCommand() {
        AutoConstants.Objective objective = m_dashboard.getObjective();
        DataLogManager.log("####### objective:" + objective);
        if(objective == null) {
          return null;
        }
        switch (objective) {
          case LEARNINGCOMMANDS:
            return LearningCommands();
        }
        return null;
      }

      public Command makeSwerveAutoCommand(String pathString) {
        ChoreoTrajectory path = Choreo.getTrajectory(pathString);

        return Choreo.choreoSwerveCommand(
            path,
            () -> m_driveTrainSubsystem.getPose(),
            new PIDController(0.9, 0, 0),
            new PIDController(0.9, 0, 0),
            new PIDController(1.1, 0, 0),
            (ChassisSpeeds speeds) -> m_driveTrainSubsystem.driveRobotRelative(speeds),
            () -> {
              if (Robot.getAllianceColor() == Alliance.Red) {
                return true;
              } else {
                return false;
              }
            },
            m_driveTrainSubsystem);
      }

      public void resetPose(String pathString) {

        ChoreoTrajectory path = Choreo.getTrajectory(pathString);

        double initRotation = path.getInitialPose().getRotation().getRadians() + Math.toRadians(getGyroResetAngle());
        if (initRotation > 2 * Math.PI) {
          initRotation -= 2 * Math.PI;
        }

        if (Robot.getAllianceColor() == Alliance.Red) {
          m_driveTrainSubsystem.swerveOnlyResetPose(new Pose2d(
              new Translation2d(path.flipped().getInitialPose().getX(), path.flipped().getInitialPose().getY()),
              // new Translation2d(15.78, 2.341),
              new Rotation2d(initRotation)));
        } else {
          m_driveTrainSubsystem.swerveOnlyResetPose(new Pose2d(
              new Translation2d(path.getInitialPose().getX(), path.getInitialPose().getY()),
              // new Translation2d(15.78, 2.341),
              new Rotation2d(initRotation)));
        }
      }

      private double getGyroResetAngle() {
        if (Robot.getAllianceColor() == Alliance.Blue) {
          return 0;
        } else {
          return 180;
        }
      }
}