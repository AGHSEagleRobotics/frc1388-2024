// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterSubsystemConstants;
import frc.robot.Constants.TransitionSubsystemConstants;
import frc.robot.commands.DeployIntakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.Constants.DriveTrainConstants;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Dashboard m_dashboard = new Dashboard();

  private final DriveTrainSubsystem m_driveTrain = new DriveTrainSubsystem(
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
      new AHRS(SerialPort.Port.kUSB) // navx
  );

  private final AutoMethod m_autoMethod = new AutoMethod(m_driveTrain, m_dashboard);

  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem(
      new CANSparkFlex(ShooterSubsystemConstants.kShooterMotor1CANID,
          MotorType.kBrushless),
      new CANSparkFlex(ShooterSubsystemConstants.kShooterMotor2CANID,
          MotorType.kBrushless));

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
      new CANSparkMax(Constants.IntakeConstants.ROLLER_MOTOR_CANID, MotorType.kBrushless),
      new CANSparkMax(Constants.IntakeConstants.LIFTER_MOTOR_CANID, MotorType.kBrushless),
      new DigitalInput(Constants.IntakeConstants.LOWER_LIMIT_DIO),
      new DigitalInput(Constants.IntakeConstants.UPPER_LIMIT_DIO),
      new DigitalInput(Constants.IntakeConstants.BEAM_BREAK_DIO));

  private final TransitionSubsystem m_transitionSubsystem = new TransitionSubsystem(
      new CANSparkMax(TransitionSubsystemConstants.MOTOR_CANID, MotorType.kBrushless));

  private final Limelight m_limelight = new Limelight(m_driveTrain);

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DriveCommand m_driveCommand = new DriveCommand(
        m_driveTrain,
        m_limelight,
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getHID().getAButton(),
        () -> m_driverController.getHID().getBButton(),
        () -> m_driverController.getHID().getXButton(),
        () -> m_driverController.getHID().getYButton(),
        () -> m_driverController.getHID().getBackButton() // test button
    );
    m_driveTrain.setDefaultCommand(m_driveCommand);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // ========MAYBE RE-ADD THESE CONTROLS?========
    // m_driverController.rightBumper().whileTrue(new RunCommand(() ->
    // m_limelight.turnToSpeaker()));
    // m_driverController.leftTrigger().whileTrue(new RunCommand(() ->
    // m_limelight.goToCenterOfSpeaker()));

    // DRIVER CONTROLS
    m_driverController.leftBumper().onTrue(new DeployIntakeCommand(m_intakeSubsystem));
    m_driverController.leftTrigger().onTrue(new RetractIntakeCommand(m_intakeSubsystem));

    // SHOOT COMMAND SEQUENCE
    m_driverController.rightTrigger().whileTrue(
        new RetractIntakeCommand(m_intakeSubsystem)
            .andThen(
                new ShooterCommand(m_shooterSubsystem)
                    .alongWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem))));

    // RESET GYRO CONTROL
    m_driverController.start().onTrue(new InstantCommand(() -> m_driveTrain.resetGyroHeading()));
    // TODO decide if reset pose is needed
    //m_driverController.start().onTrue(new InstantCommand(() -> m_driveTrain.resetPose(new Pose2d())));

    // OPERATOR CONTROLS
    m_operatorController.leftBumper().onTrue(new DeployIntakeCommand(m_intakeSubsystem));
    m_operatorController.leftTrigger().onTrue(new RetractIntakeCommand(m_intakeSubsystem));
  }

  public void setAllEncoderOffsets() {
    m_driveTrain.setAllEncoderOffsets();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoMethod.getAutonomousCommand();
  }
}
