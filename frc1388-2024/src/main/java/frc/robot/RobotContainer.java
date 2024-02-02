// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrainSubsystem;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveAutoTesting;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SerialPort;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private final SendableChooser<Command> autoChooser;
  // autoChooser = AutoBuilder.buildAutoChooser;


  public final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem(
      new SwerveModule(
          new TalonFX(Constants.DriveTrainConstants.FRONT_RIGHT_DRIVE_MOTOR_CANID), 
          new TalonFX(Constants.DriveTrainConstants.FRONT_RIGHT_ROTATION_MOTOR_CANID),
          new CANcoder(Constants.DriveTrainConstants.FRONT_RIGHT_CANCODER),
                       Preferences.getDouble(Constants.DriveTrainConstants.FRONT_RIGHT_ENCODER_OFFSET_KEY, 0)), 
      new SwerveModule(
          new TalonFX(Constants.DriveTrainConstants.FRONT_LEFT_DRIVE_MOTOR_CANID),  
          new TalonFX(Constants.DriveTrainConstants.FRONT_LEFT_ROTATION_MOTOR_CANID),
          new CANcoder(Constants.DriveTrainConstants.FRONT_LEFT_CANCODER),
                        Preferences.getDouble(Constants.DriveTrainConstants.FRONT_LEFT_ENCODER_OFFSET_KEY, 0)),
      new SwerveModule(
          new TalonFX(Constants.DriveTrainConstants.BACK_LEFT_DRIVE_MOTOR_CANID), 
          new TalonFX(Constants.DriveTrainConstants.BACK_LEFT_ROTATION_MOTOR_CANID),
          new CANcoder(Constants.DriveTrainConstants.BACK_LEFT_CANCODER),
                        Preferences.getDouble(Constants.DriveTrainConstants.BACK_LEFT_ENCODER_OFFSET_KEY, 0)),
      new SwerveModule(
          new TalonFX(Constants.DriveTrainConstants.BACK_RIGHT_DRIVE_MOTOR_CANID),  
          new TalonFX(Constants.DriveTrainConstants.BACK_RIGHT_ROTATION_MOTOR_CANID),
          new CANcoder(Constants.DriveTrainConstants.BACK_RIGHT_CANCODER),
                        Preferences.getDouble(Constants.DriveTrainConstants.BACK_RIGHT_ENCODER_OFFSET_KEY, 0)),
      new AHRS(SerialPort.Port.kUSB)
      //new ADIS16470_IMU()  
    );

    
// all those numbers should be constants review what the names should be
  private final CommandXboxController m_driverController = 
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

      NamedCommands.registerCommand("a", Commands.print("Passed marker 1"));
      autoChooser = AutoBuilder.buildAutoChooser();
      Shuffleboard.getTab("Tab 1").add(autoChooser);
      autoChooser.addOption("a", getAutonomousCommand());



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
      () -> m_driverController.getRightX(),
      // () -> getDPad()
      () -> m_driverController.getHID().getAButton(),
      () -> m_driverController.getHID().getBButton(),
      () -> m_driverController.getHID().getXButton(),
      () -> m_driverController.getHID().getYButton()

    );

    SwerveAutoTesting m_swerveAutoTesting = new SwerveAutoTesting(
      m_driveTrain, 
      () -> m_driverController.getLeftY(), 
      () -> m_driverController.getLeftX(), 
      () -> m_driverController.getRightX()
    );

    m_driveTrain.setDefaultCommand(m_driveCommand);
    // m_driveTrain.setDefaultCommand(m_swerveAutoTesting);

    
    m_driverController.start().onTrue(new InstantCommand(() -> m_driveTrain.resetGyroHeading()));
    m_driverController.start().onTrue(new InstantCommand(() -> m_driveTrain.resetPose(new Pose2d())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoDrive(m_driveTrain, -1);
    // .andThen(new AutoDrive(m_driveTrain, -1));
    // return new AutoTurn(90, m_driveTrain)
    // .andThen(new AutoDrive(m_driveTrain, 1))

    // .andThen(new AutoTurn(180, m_driveTrain))
    // .andThen(new AutoDrive(m_driveTrain, 1))

    // .andThen(new AutoTurn(270, m_driveTrain))
    // .andThen(new AutoDrive(m_driveTrain, 1))

    // .andThen(new AutoTurn(0, m_driveTrain))
    // .andThen(new AutoDrive(m_driveTrain, 1));

    // return autoChooser.getSelected();
    // return PathPlannerPath.fromPathFile("a");
    // return AutoBuilder.followPath(PathPlannerPat.h.fromPathFile("a"));

    // return new AutoDrive(m_driveTrain, 1)
    // .andThen(new AutoTurn(m_driveTrain, 90, RotationDirection.ccw))
    // .andThen(new AutoDrive(m_driveTrain, 0.5))
    // .andThen(new AutoTurn(m_driveTrain, 0, RotationDirection.cw));


    // .andThen(new AutoDrive(m_driveTrain, 2))
    // .andThen(new AutoDrive(m_driveTrain, -2))
    // .andThen(new AutoDrive(m_driveTrain, 2))
    // .andThen(new AutoDrive(m_driveTrain, -2))
    // .andThen(new AutoDrive(m_driveTrain, 2))
    // .andThen(new AutoDrive(m_driveTrain, -2))
    // .andThen(new AutoDrive(m_driveTrain, 2));
    // .andThen(new AutoTurn(m_driveTrain, 90, RotationDirection.ccw))
    // .andThen(new AutoDrive(m_driveTrain, 0.5))
    // .andThen(new AutoTurn(m_driveTrain, 0, RotationDirection.cw));

    // .andThen(new AutoTurn(m_driveTrain, 90))
    // return new PathPlannerAuto("a");
    // return PathPlannerAuto("b");
    // return null;
  }

  public int getDPad() {
    return m_driverController.getHID().getPOV();
  }
}
