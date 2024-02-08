// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
<<<<<<< HEAD
=======
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> dev
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.vision.Limelight;

public class DriveCommand extends Command {

  private final DriveTrainSubsystem m_driveTrain;
  private final Limelight m_limelight;

  private final Supplier<Double> m_leftY;
  private final Supplier<Double> m_leftX;
  private final Supplier<Double> m_rightX;
<<<<<<< HEAD
  private final Supplier<Boolean> m_start;
  private final PIDController m_pidController = new PIDController(AutoConstants.TURN_P_VALUE, AutoConstants.TURN_I_VALUE, AutoConstants.TURN_D_VALUE);
=======
  // private final Supplier<Integer> m_dPad;
  private final Supplier<Boolean> m_a;
  private final Supplier<Boolean> m_b;
  private final Supplier<Boolean> m_x;
  private final Supplier<Boolean> m_y;

>>>>>>> dev

  private boolean m_goingToAngle;
  private double m_angleSetPoint;
  private PIDController m_rotationController = new PIDController(0.003, 0, 0);

  /** Creates a new DriveCommand. */
<<<<<<< HEAD
  public DriveCommand(DriveTrainSubsystem driveTrain, Limelight limelight, Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX, Supplier<Boolean> start) {
=======
  public DriveCommand(DriveTrainSubsystem driveTrain, Supplier<Double> leftY, Supplier<Double> leftX, Supplier<Double> rightX, Supplier<Boolean> a, Supplier<Boolean> b, Supplier<Boolean> x, Supplier<Boolean> y) {
>>>>>>> dev
    m_driveTrain = driveTrain;

    m_leftY = leftY;
    m_leftX = leftX;
    m_rightX = rightX;
<<<<<<< HEAD

    m_limelight = limelight;
    m_start = start;
=======
    // m_dPad = dpad;
    m_a = a;
    m_b = b;
    m_x = x;
    m_y = y;

    m_rotationController.enableContinuousInput(0, 360);
>>>>>>> dev
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setTolerance(AutoConstants.TURN_P_TOLERANCE);
    m_pidController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
    double xVelocity = Constants.DriveTrainConstants.ROBOT_MAX_SPEED * -scale(MathUtil.applyDeadband(m_leftY.get(), Constants.DriveCommandConstants.CONTROLLER_DEADBAND), Constants.DriveCommandConstants.LEFT_STICK_SCALE);
    double yVelocity = Constants.DriveTrainConstants.ROBOT_MAX_SPEED * -scale(MathUtil.applyDeadband(m_leftX.get(), Constants.DriveCommandConstants.CONTROLLER_DEADBAND), Constants.DriveCommandConstants.LEFT_STICK_SCALE);
    // 

    double omega = 0;
    if (m_start.get())
    {
      omega = m_pidController.calculate(m_limelight.getAngleFromSpeaker());
    }
    else {
      omega = 2 * Math.PI * -scale(MathUtil.applyDeadband(m_rightX.get(), Constants.DriveCommandConstants.CONTROLLER_DEADBAND), Constants.DriveCommandConstants.RIGHT_STICK_SCALE);
    }
    
=======

    // rotation state: auto rotating, driver rotating
    // if stick input, driver rotating
    // if dpad isnt -1 auto rotating
    // if auto rotating, auto rotate
    // if auto rotating and at setpoint, driver rotating

    double leftX = MathUtil.applyDeadband(m_leftX.get(), 0.1);
    double leftY = MathUtil.applyDeadband(m_leftY.get(), 0.1);
    double rightX = MathUtil.applyDeadband(m_rightX.get(), 0.1);

    
    double xVelocity = -Constants.DriveTrainConstants.ROBOT_MAX_SPEED * scale(leftY, 2.5);
    double yVelocity = -Constants.DriveTrainConstants.ROBOT_MAX_SPEED * scale(leftX, 2.5);

    double omega = 0;
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
    if (rightX != 0) {
      m_goingToAngle = false;
      omega = -rightX;
    } else if (setAngle != -1 || m_goingToAngle) {
      m_goingToAngle = true;
      omega = m_rotationController.calculate(m_driveTrain.getAngle() - 360 + m_angleSetPoint);
    } 
    if (m_goingToAngle && Math.abs(m_driveTrain.getAngle() - 360 + m_angleSetPoint) < 5) {
      m_goingToAngle = false;
    }

    SmartDashboard.putBoolean("going to angle", m_goingToAngle);


>>>>>>> dev
    m_driveTrain.drive(xVelocity, yVelocity, omega); // max speed: 3 m/s transitional, pi rad/s (0.5 rotation/s) rotational (for now)
    // m_driveTrain.driveRobotRelative(new ChassisSpeeds(xVelocity, yVelocity, omega));

  }

  /**this method needs documentation */
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
