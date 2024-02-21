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
  private final Supplier<Boolean> m_back;
  

  private final PIDController m_limelightPIDController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);// PIDController(AutoConstants.TURN_P_VALUE, AutoConstants.TURN_I_VALUE, AutoConstants.TURN_D_VALUE);

  private boolean m_goingToAngle;
  private double m_angleSetPoint;
  private PIDController m_rotationController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);
  private boolean m_autoTracking = false;
  private boolean m_lastAutoTrackButtonPressed = false; // used for edge detection 

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveTrainSubsystem driveTrain, Limelight limelight, Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX, Supplier<Boolean> a, Supplier<Boolean> b, Supplier<Boolean> x, Supplier<Boolean> y, Supplier<Boolean> back) {
    m_driveTrain = driveTrain;
    m_limelight = limelight;

    m_leftY = leftY;
    m_leftX = leftX;
    m_rightX = rightX;
    m_a = a;
    m_b = b;
    m_x = x;
    m_y = y;
    m_back = back;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelightPIDController.setTolerance(AutoConstants.TURN_P_TOLERANCE);
    m_limelightPIDController.enableContinuousInput(0, 360);

    m_rotationController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // controller inputs    
    double leftX = MathUtil.applyDeadband(m_leftX.get(), 0.1);
    double leftY = MathUtil.applyDeadband(m_leftY.get(), 0.1);
    double rightX = -MathUtil.applyDeadband(m_rightX.get(), 0.1);
    
    // velocities from controller inputs
    double xVelocity = -DriveTrainConstants.ROBOT_MAX_SPEED * scale(leftY, 2.5);
    double yVelocity = -DriveTrainConstants.ROBOT_MAX_SPEED * scale(leftX, 2.5);
    double omega = 0;

    boolean backButton = m_back.get();

    // if back button is pressed then run auto tracking
    if (backButton && !m_lastAutoTrackButtonPressed) {
      m_autoTracking = !m_autoTracking;
    }
    m_lastAutoTrackButtonPressed = backButton;

    // setting omega value based on button bindings for rotation setpoints
    if (rightX != 0) { // default turning with stick
      omega = rightX;
      m_autoTracking = false;
      m_goingToAngle = false;
    } else if (m_a.get()) {
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
    } else if (m_y.get()) {
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
