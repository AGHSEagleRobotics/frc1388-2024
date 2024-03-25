// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterAngleSubsystemConstants;


public class ShooterAngleSubsystem extends SubsystemBase {
  private final CANSparkMax m_angleMotor;
  private final AnalogPotentiometer m_potentiometer;
  private double m_targetPosition;
  private final DriveTrainSubsystem m_driveTrain;

  /** Creates a new ShooterAngleSubsystem. */
  public ShooterAngleSubsystem(CANSparkMax angleMotor, AnalogPotentiometer potentiometer, DriveTrainSubsystem driveTrain) {
    m_angleMotor = angleMotor;
    m_potentiometer = potentiometer;
    m_driveTrain = driveTrain;
    m_targetPosition = getCurrentPosition();
  }

  public double getCurrentPosition() {
    return m_potentiometer.get();
  }

  public void setPosition(double position) {
    m_targetPosition = position;
  }

  public double getAbsouluteDistanceFromSpeaker() {
    return m_driveTrain.getAbsouluteDistanceFromSpeaker();
  }

  public double getAutoCurveFitAngle() {
    double goToAngle;
    double distance = getAbsouluteDistanceFromSpeaker();
    double distance2 = Math.pow(distance, 2);
    double distance3 = Math.pow(distance, 3);
    double distance4 = Math.pow(distance, 4);
    if ((distance > 0) && (distance < LimelightConstants.DISTANCE_FROM_APRILTAG_AUTOSHOOTER)) {
      goToAngle = LimelightConstants.QUADRATIC_AUTO_SHOOTER_A +
                  (LimelightConstants.QUADRATIC_AUTO_SHOOTER_B * distance) +
                  (LimelightConstants.QUADRATIC_AUTO_SHOOTER_C * distance2) +
                  (LimelightConstants.QUADRATIC_AUTO_SHOOTER_D * distance3) +
                  (LimelightConstants.QUADRATIC_AUTO_SHOOTER_E * distance4);
      } else {
        goToAngle = getCurrentPosition();
      }
      return goToAngle;
  }

  @Override
  public void periodic() {
    m_targetPosition = MathUtil.clamp(m_targetPosition, ShooterAngleSubsystemConstants.kShooterPositionMin, ShooterAngleSubsystemConstants.kShooterPositionMax);
    
    double currentPosition = getCurrentPosition();
    double error = m_targetPosition - currentPosition;
    double speed = error * ShooterAngleSubsystemConstants.kShooterAngleP;
    // SmartDashboard.putNumber("error", error);
    if (Math.abs(error) <= ShooterAngleSubsystemConstants.P_TOLERANCE) {
      speed = 0;
    }
    m_angleMotor.set(speed);
    SmartDashboard.putNumber("shooter angle subsystem/error", error);
    SmartDashboard.putNumber("shooter angle subsystem/shooter speed", speed);


    // if (currentPosition < m_targetPosition - 0.01) {
    // m_angleMotor.set(1);
    // }
    // else if (currentPosition > m_targetPosition + 0.01) {
    //   m_angleMotor.set(-1);
    // }
    SmartDashboard.putNumber("shooter angle subsystem/potentiometerPosition", currentPosition);
    SmartDashboard.putNumber("shooter angle subsystem/target position", m_targetPosition);
    SmartDashboard.putNumber("shooter angle subsystem/speed", speed);
    // This method will be called once per scheduler run
  }
}
