// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveAutoTesting;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Power;
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
  // public final ShooterSubsystem m_shooter = new ShooterSubsystem(
  //   new CANSparkMax(ShooterSubsystemConstants.kShooterMotor1CANID, MotorType.kBrushless),
  //   new CANSparkMax(ShooterSubsystemConstants.kShooterMotor2CANID, MotorType.kBrushless)
  //   );

    public final LoggingSubsystem m_logger = new LoggingSubsystem();
    

  private final SendableChooser<Command> autoChooser;
  // autoChooser = AutoBuilder.buildAutoChooser;

  private final Dashboard m_dashboard = new Dashboard();

  public final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem(
      new SwerveModule(
          new TalonFX(DriveTrainConstants.FRONT_RIGHT_DRIVE_MOTOR_CANID), 
          new TalonFX(DriveTrainConstants.FRONT_RIGHT_ROTATION_MOTOR_CANID),
          new CANcoder(DriveTrainConstants.FRONT_RIGHT_CANCODER),
                       Preferences.getDouble(DriveTrainConstants.FRONT_RIGHT_ENCODER_OFFSET_KEY, 0)), 
      new SwerveModule(
          new TalonFX(DriveTrainConstants.FRONT_LEFT_DRIVE_MOTOR_CANID),  
          new TalonFX(DriveTrainConstants.FRONT_LEFT_ROTATION_MOTOR_CANID),
          new CANcoder(DriveTrainConstants.FRONT_LEFT_CANCODER),
                        Preferences.getDouble(DriveTrainConstants.FRONT_LEFT_ENCODER_OFFSET_KEY, 0)),
      new SwerveModule(
          new TalonFX(DriveTrainConstants.BACK_LEFT_DRIVE_MOTOR_CANID), 
          new TalonFX(DriveTrainConstants.BACK_LEFT_ROTATION_MOTOR_CANID),
          new CANcoder(DriveTrainConstants.BACK_LEFT_CANCODER),
                        Preferences.getDouble(DriveTrainConstants.BACK_LEFT_ENCODER_OFFSET_KEY, 0)),
      new SwerveModule(
          new TalonFX(DriveTrainConstants.BACK_RIGHT_DRIVE_MOTOR_CANID),  
          new TalonFX(DriveTrainConstants.BACK_RIGHT_ROTATION_MOTOR_CANID),
          new CANcoder(DriveTrainConstants.BACK_RIGHT_CANCODER),
                        Preferences.getDouble(DriveTrainConstants.BACK_RIGHT_ENCODER_OFFSET_KEY, 0)),
      new AHRS(SerialPort.Port.kUSB)
      //new ADIS16470_IMU()  
    );
    
  public final IntakeSubsystem m_intake = new IntakeSubsystem(
    new CANSparkMax(Constants.IntakeConstants.ROLLER_MOTOR_CANID, MotorType.kBrushless), 
    new CANSparkMax(Constants.IntakeConstants.LIFTER_MOTOR_CANID, MotorType.kBrushless), 
    new DigitalInput(Constants.IntakeConstants.LOWER_LIMIT_DIO),  
    new DigitalInput(Constants.IntakeConstants.UPPER_LIMIT_DIO),
    new DigitalInput(Constants.IntakeConstants.BEAM_BREAK_DIO)
  );

  // private final TransitionSubsystem m_transitionSubsystem = new TransitionSubsystem(new CANSparkMax(21, MotorType.kBrushed));

  // all those numbers should be constants review what the names should be
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

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
    // m_driveTrain.setDefaultCommand(m_swerveAutoTesting)
    
    /* driver and operator button binds for left bumper */

    m_driverController.leftBumper().onTrue(new DeployIntakeCommand(m_intake));
    m_operatorController.leftBumper().onTrue(new DeployIntakeCommand(m_intake));

    /** driver and operator button bindings for left trigger */

    m_driverController.leftTrigger().onTrue(new RetractIntakeCommand(m_intake));
    m_operatorController.leftTrigger().onTrue(new RetractIntakeCommand(m_intake));


    m_driverController.rightTrigger().whileTrue(
      new RetractIntakeCommand(m_intake)
      .andThen(
        // new ShooterCommand(m_shooter)
        // .alongWith(new FeedShooter(null, m_intake))
        new FeedShooter(m_intake)
      )
    );
    
    m_driverController.start().onTrue(new InstantCommand(() -> m_driveTrain.resetGyroHeading()));
    m_driverController.start().onTrue(new InstantCommand(() -> m_driveTrain.resetPose(new Pose2d())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new AutoDrive(4, m_driveTrain)
    .andThen(new AutoTurn(90, m_driveTrain))
    .andThen(new AutoDrive(1, m_driveTrain))
    .andThen(new AutoTurn(90, m_driveTrain))
    .andThen(new AutoDrive(4, m_driveTrain))
    .andThen(new AutoTurn(90, m_driveTrain))
    .andThen(new AutoDrive(1, m_driveTrain));
  }

  public int getDPad() {
    return m_driverController.getHID().getPOV();
  }
}
