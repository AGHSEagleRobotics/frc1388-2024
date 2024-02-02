// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.vision.Limelight;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  

  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem(
      new SwerveModule(
          new TalonFX(1),
          new TalonFX(5),
          new CANcoder(9),
          282), 
      new SwerveModule(
          new TalonFX(2),
          new TalonFX(6),
          new CANcoder(10),
          203),
      new SwerveModule(
          new TalonFX(3),
          new TalonFX(7),
          new CANcoder(11),
          36),
      new SwerveModule(
          new TalonFX(4),
          new TalonFX(8),
          new CANcoder(12),
          167),
      new AHRS(SerialPort.Port.kUSB)
      // new ADIS16470_IMU()
      
    );
  private AHRS m_navxGyro;
    Limelight m_limelight = new Limelight(m_driveTrain, m_navxGyro);
// all those numbers should be constants review what the names should be
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    DriveCommand m_driveCommand = new DriveCommand(
      m_driveTrain, 
      () -> m_driverController.getLeftY(), 
      () -> m_driverController.getLeftX(), 
      () -> m_driverController.getRightX()
    );

    m_driveTrain.setDefaultCommand(m_driveCommand);

    
    m_driverController.a().onTrue(new InstantCommand(() -> m_driveTrain.resetGyroHeading()));
    m_driverController.a().onTrue(new InstantCommand(() -> m_driveTrain.resetPose(new Pose2d())));
    m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_limelight.turnToSpeaker()));
    m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_limelight.goToSpeaker()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    // V Auto code V
    return new WaitCommand(5);
    // ^ Auto code ^
  }
}
