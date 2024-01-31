// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.RetractIntakeCommand;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    
  public final IntakeSubsystem m_intake = new IntakeSubsystem(
    new CANSparkMax(Constants.IntakeConstants.ROLLER_MOTOR_CANID, MotorType.kBrushless), 
    new CANSparkMax(Constants.IntakeConstants.LIFTER_MOTOR_CANID, MotorType.kBrushless), 
    new DigitalInput(Constants.IntakeConstants.LOWER_LIMIT_DIO),  
    new DigitalInput(Constants.IntakeConstants.UPPER_LIMIT_DIO));

  // all those numbers should be constants review what the names should be
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

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
    
    /* driver and operator button binds for left bumper */

    m_driverController.leftBumper().onTrue(new DeployIntakeCommand(m_intake));
    m_operatorController.leftBumper().onTrue(new DeployIntakeCommand(m_intake));

    /** driver and operator button bindings for left trigger */

    m_driverController.leftTrigger().onTrue(new RetractIntakeCommand(m_intake));
    m_operatorController.leftTrigger().onTrue(new RetractIntakeCommand(m_intake));


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
