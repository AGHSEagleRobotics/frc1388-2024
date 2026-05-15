// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkFlex m_bottomShooterMotor;
  private final SparkFlex m_topShooterMotor;

  private final SparkClosedLoopController m_bottomShooterMotorPIDController;
  private final SparkClosedLoopController m_topShooterMotorPIDController;

  private final RelativeEncoder m_bottomShooterEncoder;
  private final RelativeEncoder m_topMotorEncoder;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(SparkFlex bottomShooterMotor, SparkFlex topShooterMotor) {
    m_bottomShooterMotor = bottomShooterMotor;
    m_topShooterMotor = topShooterMotor;

    m_bottomShooterEncoder = m_bottomShooterMotor.getEncoder();
    m_topMotorEncoder = m_topShooterMotor.getEncoder();

    SparkFlexConfig bottomConfig = new SparkFlexConfig();
    bottomConfig.idleMode(IdleMode.kCoast).inverted(true);
    bottomConfig.closedLoop.pidf(
        ShooterConstants.SHOOTER_MOTOR_P,
        ShooterConstants.SHOOTER_MOTOR_I,
        ShooterConstants.SHOOTER_MOTOR_D,
        ShooterConstants.SHOOTER_MOTOR_FF);
    m_bottomShooterMotor.configure(bottomConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkFlexConfig topConfig = new SparkFlexConfig();
    topConfig.idleMode(IdleMode.kCoast).inverted(false);
    topConfig.closedLoop.pidf(
        ShooterConstants.SHOOTER_MOTOR_P,
        ShooterConstants.SHOOTER_MOTOR_I,
        ShooterConstants.SHOOTER_MOTOR_D,
        ShooterConstants.SHOOTER_MOTOR_FF);
    m_topShooterMotor.configure(topConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_bottomShooterMotorPIDController = m_bottomShooterMotor.getClosedLoopController();
    m_topShooterMotorPIDController = m_topShooterMotor.getClosedLoopController();
  }

  public void setPower(double power) {
    m_bottomShooterMotor.set(power);
    m_topShooterMotor.set(power);
  }

  /**
   * sets bottom motor velocity
   * 
   * @param rpm setting motor to rpm velocity
   */
  private void setBottomMotorVelocity(double rpm) {
    m_bottomShooterMotorPIDController.setReference(rpm, ControlType.kVelocity);
  }

    /**
   * sets top motor velocity
   * 
   * @param rpm setting motor to rpm velocity
   */
  private void setTopMotorVelocity(double rpm) {
    m_topShooterMotorPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public double getBottomMotorVelocity() {
    return m_bottomShooterEncoder.getVelocity();
  }

  public double getTopMotorVelocity() {
    return m_topMotorEncoder.getVelocity();
  }

  public void setShooterRPM(double rpm) {
    setBottomMotorVelocity(rpm);
    setTopMotorVelocity(rpm);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double bottomMotorVelocity = getBottomMotorVelocity();
    double topMotorVelocity = getTopMotorVelocity();

    SmartDashboard.putNumber("Shooter/bottom rpm", bottomMotorVelocity);
    SmartDashboard.putNumber("Shooter/top rpm", topMotorVelocity);
  }
}
