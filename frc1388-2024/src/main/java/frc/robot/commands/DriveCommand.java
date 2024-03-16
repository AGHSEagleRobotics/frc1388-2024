// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.vision.Limelight;

public class DriveCommand extends Command {

  private final DriveTrainSubsystem m_driveTrain;
  private final Limelight m_limelight;

  private final Supplier<Double> m_leftY;
  private final Supplier<Double> m_leftX;
  private final Supplier<Double> m_rightX;
  private final Supplier<Boolean> m_a;
  private final Supplier<Boolean> m_b;
  private final Supplier<Boolean> m_x;
  private final Supplier<Boolean> m_y;
  private final Supplier<Boolean> m_rightStick;
  

  private final PIDController m_limelightPIDController = new PIDController(LimelightConstants.TURN_P_VALUE_AUTO_TRACKING, LimelightConstants.TURN_I_VALUE_AUTO_TRACKING, LimelightConstants.TURN_D_VALUE_AUTO_TRACKING);// PIDController(AutoConstants.TURN_P_VALUE, AutoConstants.TURN_I_VALUE, AutoConstants.TURN_D_VALUE);

  private boolean m_goingToAngle;
  private double m_angleSetPoint;
  private PIDController m_rotationController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);
  private boolean m_autoTracking = false;
  private boolean m_lastAutoTrackButtonPressed = false; // used for edge detection 

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveTrainSubsystem driveTrain, Limelight limelight, Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX, Supplier<Boolean> a, Supplier<Boolean> b, Supplier<Boolean> x, Supplier<Boolean> y, Supplier<Boolean> rightStick) {
    m_driveTrain = driveTrain;
    m_limelight = limelight;

    m_leftY = leftY;
    m_leftX = leftX;
    m_rightX = rightX;
    m_a = a;
    m_b = b;
    m_x = x;
    m_y = y;
    m_rightStick = rightStick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TEMP removed
    // m_limelightPIDController.setTolerance(AutoConstants.TURN_P_TOLERANCE);
    // m_limelightPIDController.enableContinuousInput(0, 360);

    m_rotationController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // controller inputs    
    double leftX = MathUtil.applyDeadband(m_leftX.get(), DriveTrainConstants.CONTROLLER_DEADBAND);
    double leftY = MathUtil.applyDeadband(m_leftY.get(), DriveTrainConstants.CONTROLLER_DEADBAND);
    double rightX = -MathUtil.applyDeadband(m_rightX.get(), DriveTrainConstants.CONTROLLER_DEADBAND);
    
    // velocities from controller inputs
    double xVelocity = -DriveTrainConstants.ROBOT_MAX_SPEED * scale(leftY, DriveTrainConstants.LEFT_STICK_SCALE);
    double yVelocity = -DriveTrainConstants.ROBOT_MAX_SPEED * scale(leftX, DriveTrainConstants.LEFT_STICK_SCALE);
    /** angular velocity */
    double omega = 0;

    boolean rightStickButton = m_rightStick.get();

    // if back button is pressed then run auto tracking
    if (rightStickButton && !m_lastAutoTrackButtonPressed) {
      m_autoTracking = !m_autoTracking;
    }
    m_lastAutoTrackButtonPressed = rightStickButton;

    // setting omega value based on button bindings for rotation setpoints
    if (rightX != 0) { // default turning with stick
      omega = scale(rightX, 2.5);
      m_autoTracking = false;
      m_goingToAngle = false;
    } else if (m_y.get()) {
      m_goingToAngle = true;
      m_autoTracking = false;
      m_angleSetPoint = 180;
    } else if (m_b.get()) {
      m_goingToAngle = true;
      m_autoTracking = false;
      m_angleSetPoint = 240;
    } else if (m_x.get()) {
      m_goingToAngle = true;
      m_autoTracking = false;
      m_angleSetPoint = 120;
    } else if (m_a.get()) {
      m_autoTracking = false;
      m_goingToAngle = true;
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        m_angleSetPoint = 90;
      } else {
        m_angleSetPoint = 270;
      }
    }

    if (m_goingToAngle) {// a/b/x/y rotation setpoints
      omega = m_rotationController.calculate(m_driveTrain.getAngle() - 360 + m_angleSetPoint);

      if (Math.abs(m_driveTrain.getAngle() - 360 + m_angleSetPoint) < 5) {
        m_goingToAngle = false;
      }
    }

    else if (m_autoTracking) {
      if (m_limelight.getAprilTagID() == 4 || m_limelight.getAprilTagID() == 7) {
      double speed = m_limelightPIDController.calculate(m_limelight.getAngleFromSpeaker());
      omega = speed;
      }
    }

    SmartDashboard.putBoolean("going to angle", m_goingToAngle);

    m_driveTrain.drive(xVelocity, yVelocity, omega); // max speed: 3 m/s transitional, pi rad/s (0.5 rotation/s) rotational (for now)
  }

  /** This method uses the tangent function to scale the input
   * <p>
   * Example: <a>https://www.desmos.com/calculator/kntlmrgprn</a>
   * 
   * @param in the raw input
   * @param scale the scaling number (represented as a in the example)
   * @return the scaled output
   */
  private double scale(double in, double scale) {
    return Math.tan(in * Math.atan(scale)) / scale;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
