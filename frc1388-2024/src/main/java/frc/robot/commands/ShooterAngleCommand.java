// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterAngleSubsystemConstants;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.vision.Limelight;

public class ShooterAngleCommand extends Command {

  private final Supplier<Boolean> m_xButton;
  private final Supplier<Boolean> m_yButton;
  private final Supplier<Boolean> m_aButton;
  private final Supplier<Boolean> m_bButton;
  private final Supplier<Double> m_leftY;
  private final Supplier<Boolean> m_start;
  private final ShooterAngleSubsystem m_shooterAngleSubsystem;
  private final Limelight m_limelight;
  private boolean m_manualMode;
  private boolean m_autoMode;

  /** Creates a new ShooterAngleCommand. */
  public ShooterAngleCommand(Supplier<Boolean> xButton, Supplier<Boolean> yButton, Supplier<Boolean> aButton, Supplier<Boolean> bButton,
      Supplier<Double> leftY, Supplier<Boolean> start, ShooterAngleSubsystem shooterAngleSubsystem,
      Limelight limelight) {

    m_xButton = xButton;
    m_yButton = yButton;
    m_aButton = aButton;
    m_bButton = bButton;
    m_leftY = leftY;
    m_start = start;
    m_shooterAngleSubsystem = shooterAngleSubsystem;
    m_limelight = limelight;
    m_manualMode = false;
    m_autoMode = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooterAngleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterAngleSubsystem.setPosition(m_shooterAngleSubsystem.getCurrentPosition());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = MathUtil.applyDeadband(m_leftY.get(), DriveTrainConstants.MANUAL_CONTROL_ANGLE_DEADBAND);

      double goToAngle = m_shooterAngleSubsystem.getCurrentPosition();

      double distance;
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        distance = m_limelight.getDistanceOfTagId(4);
      } else {
        distance = m_limelight.getDistanceOfTagId(7);
      }

      double distance2 = distance * distance;

      
      // mycurvefit numbers for quadratic interpolation
      if ((distance > 0) && (distance < LimelightConstants.DISTANCE_FROM_APRILTAG_AUTOSHOOTER)) {
      goToAngle = LimelightConstants.QUADRATIC_AUTO_SHOOTER_A +
                  (LimelightConstants.QUADRATIC_AUTO_SHOOTER_B * distance) +
                  (LimelightConstants.QUADRATIC_AUTO_SHOOTER_C * distance2);
      }

      // if ( (distance > 0) && (distance < LimelightConstants.DISTANCE_FROM_APRILTAG_POSITIONB)) {
      // goToAngle = (LimelightConstants.SLOPE_MATH_SUBLIFER_TO_POSITIONB * distance) + LimelightConstants.SHOOTER_OFFSET_SUBTOB;
      //  goToAngle = MathUtil.clamp(goToAngle, ShooterAngleSubsystemConstants.kShooterPositionNoteB, ShooterAngleSubsystemConstants.kShooterPositionUp);
      // }
      // else if ((distance > LimelightConstants.DISTANCE_FROM_APRILTAG_POSITIONB) && (distance < LimelightConstants.DISTANCE_FROM_APRILTAG_PODIUM)) {
      //   goToAngle = (LimelightConstants.SLOPE_MATH_POSITIONB_TO_PODIUM * distance) + LimelightConstants.SHOOTER_OFFSET_B_TO_POD;
      //   goToAngle = MathUtil.clamp(goToAngle, ShooterAngleSubsystemConstants.kShooterPositionDown, ShooterAngleSubsystemConstants.kShooterPositionNoteB);
      // }
      // else if ((distance > LimelightConstants.DISTANCE_FROM_APRILTAG_PODIUM) && (distance < LimelightConstants.DISTANCE_FROM_APRILTAG_WING)) {
      //   goToAngle = (LimelightConstants.SLOPE_MATH_PODIUM_TO_WING * distance) + LimelightConstants.SHOOTER_OFFSET_WING;
      //   goToAngle = MathUtil.clamp(goToAngle, ShooterAngleSubsystemConstants.kShooterPositionWing, ShooterAngleSubsystemConstants.kShooterPositionDown);
      // }
   
    SmartDashboard.putNumber("Shooter/Auto Tracking Shooter Angle", goToAngle); 

    boolean startButton = m_start.get();
    if (startButton) {
      m_autoMode = !m_autoMode;
      if (m_autoMode) {
        m_manualMode = false;
      }
    }

    if (m_yButton.get()) {
      m_manualMode = false;
      m_autoMode = false;
      m_shooterAngleSubsystem.setPosition(ShooterAngleSubsystemConstants.kShooterPositionSpeaker);
    } else if (m_aButton.get()) {
      m_manualMode = false;
      m_autoMode = false;
      m_shooterAngleSubsystem.setPosition(ShooterAngleSubsystemConstants.kShooterPositionPodium);
    } else if (m_bButton.get()) {
      m_manualMode = false;
      m_autoMode = false;
      m_shooterAngleSubsystem.setPosition(ShooterAngleSubsystemConstants.kShooterPositionMax);
    } else if (m_xButton.get()) {
      m_manualMode = false;
      m_autoMode = false;
      m_shooterAngleSubsystem.setPosition(ShooterAngleSubsystemConstants.kShooterPositionDrewSaucyShot);
    } else if (leftY < 0) {
      m_manualMode = true;
      m_autoMode = false;
      m_shooterAngleSubsystem.setPosition(ShooterAngleSubsystemConstants.kShooterPositionMax);
    } else if (leftY > 0) {
      m_manualMode = true;
      m_autoMode = false;
      m_shooterAngleSubsystem.setPosition(ShooterAngleSubsystemConstants.kShooterPositionMin);
    } else if (m_manualMode == true) {
      m_shooterAngleSubsystem.setPosition(m_shooterAngleSubsystem.getCurrentPosition());
    } else if (m_autoMode == true) {
      
        m_shooterAngleSubsystem.setPosition(goToAngle);
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
