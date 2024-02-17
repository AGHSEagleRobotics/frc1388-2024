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
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.vision.Limelight;

public class DriveCommand extends Command {

  private final DriveTrainSubsystem m_driveTrain;
  private final Limelight m_limelight;

  private final Supplier<Double> m_leftY;
  private final Supplier<Double> m_leftX;
  private final Supplier<Double> m_rightX;
  private final Supplier<Boolean> m_backButton;
  private final Supplier<Boolean> m_a;
  private final Supplier<Boolean> m_b;
  private final Supplier<Boolean> m_x;
  private final Supplier<Boolean> m_y;
  
  private final PIDController m_limelightPIDController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);// PIDController(AutoConstants.TURN_P_VALUE, AutoConstants.TURN_I_VALUE, AutoConstants.TURN_D_VALUE);

  private boolean m_goingToAngle;
  private double m_angleSetPoint;
  private PIDController m_rotationController = new PIDController(AutoConstants.TURN_P_VALUE, 0, 0);

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
    m_backButton = back;

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

    
    if (rightX != 0) { // default turning with stick
      omega = rightX;
    } else if (m_backButton.get()) { // limelight auto tracking
      omega = m_limelightPIDController.calculate(m_limelight.getAngleFromSpeaker());
    } else { // a/b/x/y rotation setpoints
      int setAngle = 0;
      if (m_a.get()) {
        setAngle = 180;
      } else if (m_b.get()) {
        setAngle = 240;
      } else if (m_x.get()) {
        setAngle = 120;
      } else if (m_y.get()) {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
          setAngle = 90;
        } else {
          setAngle = 270;
        }
      } else {
        setAngle = -1; // default, -1 indicates no set point
      }

      if (setAngle != -1) {
        m_angleSetPoint = setAngle;
      }
       if (setAngle != -1 || m_goingToAngle) {
        m_goingToAngle = true;
        omega = m_rotationController.calculate(m_driveTrain.getAngle() - 360 + m_angleSetPoint);
      } 
      if (m_goingToAngle && Math.abs(m_driveTrain.getAngle() - 360 + m_angleSetPoint) < 5) {
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
