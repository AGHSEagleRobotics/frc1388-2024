// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LidarSubsystem extends SubsystemBase {
  LaserCan m_lasercan = new LaserCan(29);
  /** Creates a new LidarSubsystem. */
  public LidarSubsystem() {
    
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("lasercan/getMeasurement()", m_lasercan.getMeasurement().distance_mm);
    SmartDashboard.putBoolean("lasercan/validMeasurement", m_lasercan.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    SmartDashboard.putBoolean("lasercan/outOfBounds", m_lasercan.getMeasurement().status == LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS);
    }


  }
  


