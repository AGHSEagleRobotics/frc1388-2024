// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LidarSubsystem extends SubsystemBase {
  LaserCan m_lasercan = new LaserCan(29);
  /** Creates a new LidarSubsystem. */
  public LidarSubsystem() {
    
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LaserCan.Measurement measurement = m_lasercan.getMeasurement();
    if (measurement != null) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    }
  }

}
