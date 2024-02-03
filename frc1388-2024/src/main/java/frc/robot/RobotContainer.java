// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LoggingSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterCommand;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.math.geometry.Pose2d;
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
  public final ShooterSubsystem m_shooter = new ShooterSubsystem(
    new CANSparkMax(ShooterSubsystemConstants.kShooterMotor1CANID, MotorType.kBrushless),
    new CANSparkMax(ShooterSubsystemConstants.kShooterMotor2CANID, MotorType.kBrushless)
    );

    public final LoggingSubsystem m_logger = new LoggingSubsystem();
    
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

    m_driverController.rightTrigger().onTrue(new ShooterCommand(m_shooter));
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
