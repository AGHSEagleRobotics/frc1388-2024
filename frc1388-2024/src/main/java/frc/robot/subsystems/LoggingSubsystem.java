// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class supports logging of miscellaneous data that's not associated with
 * another specific command or subsystem.
 */
public class LoggingSubsystem extends SubsystemBase {

  // Log entries
  // private DataLog m_log = DataLogManager.getLog();

  /** Creates a new LoggingSubsystem. */
  public LoggingSubsystem() {

    // Log RoboRIO file system usage
    File logPath = new File("home/lvuser");
    long totalSpace = logPath.getTotalSpace();
    long freeSpace = logPath.getUsableSpace();
    long fileUsage = ((totalSpace - freeSpace) * 100 / totalSpace);
    DataLogManager.log("RoboRIO disk usage: " + fileUsage + "%");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
