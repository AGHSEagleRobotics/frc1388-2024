// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.choreo.lib.*;

import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.vision.Limelight;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransitionConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.Eject;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.IntakeTransitionCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SwerveAutoTesting;
import frc.robot.commands.IntakeTransitionCommand.IntakeTransState;
import frc.robot.commands.ShooterAngleCommand;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransitionSubsystem;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterAngleSubsystemConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  private final Limelight m_limelight = new Limelight("limelight-shooter", "limelight-intake");


  Field2d m_Field2d = new Field2d();

  private final boolean option8 = true;
  private final Dashboard m_dashboard = new Dashboard();
    
  public final ShooterSubsystem m_shooterSubsystem;


    
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
      new AHRS(SerialPort.Port.kUSB), // navx
      m_limelight
  );

  public final ShooterAngleSubsystem m_shooterAngleSubsystem;
  
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(
      new CANSparkMax(IntakeConstants.ROLLER_MOTOR_CANID, MotorType.kBrushless),
      new CANSparkMax(IntakeConstants.LIFTER_MOTOR_CANID, MotorType.kBrushless),
      new DigitalInput(IntakeConstants.LOWER_LIMIT_DIO),
      new DigitalInput(IntakeConstants.UPPER_LIMIT_DIO),
      new DigitalInput(IntakeConstants.BEAM_BREAK_DIO));

  private final TransitionSubsystem m_transitionSubsystem = new TransitionSubsystem(
    new CANSparkMax(TransitionConstants.TRANSITION_MOTOR_CANID, MotorType.kBrushless),
    new DigitalInput(4)
  );

  // private final LEDSubsystem m_ledSubsystem = new LEDSubsystem(new PWMSparkMax(0));
    
  private final CommandXboxController m_driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController m_operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  

    private final AutoMethod m_autoMethod;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // ChoreoTrajectory path = Choreo.getTrajectory("path1");

    // m_Field2d.getObject("path1").setPoses(path.getInitialPose(), path.getFinalPose());
    // m_Field2d.getObject("pathposes").setPoses(path.getPoses());
    // SmartDashboard.putData(m_Field2d);

    if (option8) {
      m_shooterSubsystem = new ShooterSubsystem(
          new CANSparkFlex(ShooterConstants.BOTTOM_SHOOTER_MOTOR_CANID,
              MotorType.kBrushless),
          new CANSparkFlex(ShooterConstants.TOP_SHOOTER_MOTOR_CANID,
              MotorType.kBrushless));

      m_shooterAngleSubsystem = new ShooterAngleSubsystem(
          new CANSparkMax(ShooterAngleSubsystemConstants.kShooterAngleMotorCANID, MotorType.kBrushed),
          new AnalogPotentiometer(ShooterAngleSubsystemConstants.kPotentiometerAnalogIN), m_driveTrain);

      m_autoMethod = new AutoMethod(m_driveTrain, m_dashboard, m_shooterSubsystem, m_intakeSubsystem,
          m_transitionSubsystem, m_shooterAngleSubsystem, m_limelight);

    } else {
      m_shooterSubsystem = null;
      m_shooterAngleSubsystem = null;
      m_autoMethod = null;
    }



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
      () -> m_driverController.getHID().getRightStickButton()
    );
      
    m_driveTrain.setDefaultCommand(m_driveCommand);

    if (option8) {
    ShooterAngleCommand m_ShooterAngleCommand = new ShooterAngleCommand(
        () -> m_operatorController.getHID().getXButton(),
        () -> m_operatorController.getHID().getYButton(),
        () -> m_operatorController.getHID().getAButton(),
        () -> m_operatorController.getHID().getBButton(),
        () -> m_operatorController.getLeftY(),
        () -> m_operatorController.getHID().getStartButton(),
        m_shooterAngleSubsystem, m_limelight);
        
    m_shooterAngleSubsystem.setDefaultCommand(m_ShooterAngleCommand);
    }

    SwerveAutoTesting m_swerveAutoTesting = new SwerveAutoTesting(
        m_driveTrain,
        () -> m_driverController.getLeftY(),
        () -> m_driverController.getLeftX(),
        () -> m_driverController.getRightX());
        
    m_driveTrain.setDefaultCommand(m_driveCommand);
        
    m_intakeSubsystem.setDefaultCommand(new RetractIntakeCommand(m_intakeSubsystem, m_transitionSubsystem, false));

    // Configure the trigger bindings
    configureBindings();
  }

    public void setBrakeMode(boolean brakeMode){
      m_driveTrain.setBrakeMode(brakeMode);
      m_intakeSubsystem.setBrakeMode(brakeMode);
      m_transitionSubsystem.setBrakeMode(brakeMode);
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
    DriverStation.silenceJoystickConnectionWarning(true);
    // ========MAYBE RE-ADD THESE CONTROLS?========
    // m_driverController.rightBumper().whileTrue(new RunCommand(() ->
    // m_limelight.turnToSpeaker()));
    // m_driverController.leftTrigger().whileTrue(new RunCommand(() ->
    // m_limelight.goToCenterOfSpeaker()));

    // DRIVER CONTROLS

    
    if (option8) {
    m_driverController.leftBumper().onTrue(new IntakeTransitionCommand
      (IntakeTransState.DEPLOYING,
      true,
      true, m_intakeSubsystem,
      m_transitionSubsystem,
      m_limelight,
      m_operatorController,
      m_driverController));

    m_driverController.leftTrigger().onTrue(new IntakeTransitionCommand
      (IntakeTransState.RETRACTING,
      false, 
      true,
      m_intakeSubsystem, 
      m_transitionSubsystem, 
      m_limelight));
    }    
    // SHOOT SPEAKER COMMAND SEQUENCE
    if (option8) {
    m_driverController.rightTrigger(0.9).whileTrue(
      new IntakeTransitionCommand(IntakeTransState.RETRACTING, false, true, m_intakeSubsystem, m_transitionSubsystem, m_limelight)
      .andThen(
        new ShooterCommand(ShooterConstants.SPEAKER_SHOT_RPM, m_shooterSubsystem) // speaker shot rmp
        .alongWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem))
      )
    );
    }
    // SHOOT AMP COMMAND SEQUENCE
    if (option8) {

    m_driverController.rightBumper().whileTrue(
          new IntakeTransitionCommand(IntakeTransState.RETRACTING, false, true, m_intakeSubsystem, m_transitionSubsystem,
              m_limelight)
      .andThen(
        new ShooterCommand(ShooterConstants.AMP_SHOT_RPM, m_shooterSubsystem) // amp shot rmp
                      .alongWith(new FeedShooter(m_transitionSubsystem, m_intakeSubsystem))));
    }

    // RESET GYRO CONTROL

    m_driverController.start().onTrue(new InstantCommand(() -> m_driveTrain.limelightResetGyro()));
    // TODO decide if reset pose is needed
    //m_driverController.start().onTrue(new InstantCommand(() -> m_driveTrain.resetPose(new Pose2d())));

    // OPERATOR CONTROLS
    if (option8) {
    m_operatorController.leftBumper().onTrue(new IntakeTransitionCommand
      (IntakeTransState.DEPLOYING,
      true,
      true,
      m_intakeSubsystem,
      m_transitionSubsystem,
      m_limelight,
      m_operatorController, 
      m_driverController));
  
    m_operatorController.leftTrigger().onTrue(new IntakeTransitionCommand
      (IntakeTransState.RETRACTING,
      false,
      true,
      m_intakeSubsystem,
      m_transitionSubsystem, 
      m_limelight));
  
    m_operatorController.rightBumper().whileTrue(new Eject
      (m_intakeSubsystem, 
      m_transitionSubsystem));
  
    m_operatorController.rightTrigger().onTrue(new ShooterCommand
      (ShooterConstants.SPEAKER_SHOT_RPM, 
      m_shooterSubsystem));
    }
    
    // TODO test what these 2 will do and if it works, especially if we need to input values to linepuwithapriltag
    // m_operatorController.back().whileTrue(new GoToNote(m_driveTrain, m_limelight, m_intakeSubsystem));
    // m_operatorController.b().whileTrue(new LineUpWithAprilTag(m_driveTrain, m_limelight, 0, 0));
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
    return new WaitCommand(m_dashboard.getWaitTime()).andThen(m_autoMethod.getAutonomousCommand()) ;
  
  }

  // public void autonomousInit() {
  //   m_limelight.setAllianceColor(DriverStation.getAlliance().get());
  // }

  // public void teleopInit() {
  //   m_limelight.setAllianceColor(DriverStation.getAlliance().get());
  // }

  public void resetPose() {
    m_driveTrain.resetPose(new Pose2d());
    new Pose2d(0, 0, m_driveTrain.getGyroHeading());
  }

   public void limelightResetGyro() {
    if (m_limelight.getApriltagTargetFound()) {
    m_driveTrain.limelightResetGyro();
    }
  }
    
  public void resetGyro() {
    m_driveTrain.resetGyroHeading(180);
  }

  public boolean getDPadUp() {
    return m_driverController.getHID().getPOV() == 0;
  }
  public boolean getDPadDown() {
    return m_driverController.getHID().getPOV() == 180;
  }
  public boolean getDPadRight() {
    return m_driverController.getHID().getPOV() == 90;
  }

  private double getGyroResetAngle() {
    if (Robot.getAllianceColor() == Alliance.Blue) {
      return 180;
    } else {
      return 0;
    }
  }

}
