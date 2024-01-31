// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Swerve {
        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(0.05, 0, 0), // Translation constants 
      new PIDConstants(0.05, 0, 0), // Rotation constants 
      4.5, 
      new Translation2d(FieldConstants.ROBOT_WIDTH / 2, FieldConstants.ROBOT_LENGTH / 2).getNorm(), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
  }

  public static class SwerveModuleConstants { 
    public static final double DIST_PER_TICK = (1.0 / 6.75) * (0.3192); // ask calvin about the math

    public static final double kDriveMotorP = 5;
    public static final double kDriveMotorI = 0;
    public static final double kDriveMotorD = 0;

    public static final double kRotationP = 5;
    public static final double kRotationI = 0;
    public static final double kRotationD = 0;

    public static final double kRotationTolerance = 5;
  }

  public static class DriveTrainConstants {
    public static final double ROBOT_MAX_SPEED = 0.25; //meters per second

    public static final int FRONT_RIGHT_DRIVE_MOTOR_CANID = 1;
    public static final int FRONT_RIGHT_ROTATION_MOTOR_CANID = 5;
    public static final int FRONT_RIGHT_CANCODER = 9;
    public static final String FRONT_RIGHT_ENCODER_OFFSET_KEY = "2024/frontRightEncoderOffset";

    public static final int FRONT_LEFT_DRIVE_MOTOR_CANID = 2;
    public static final int FRONT_LEFT_ROTATION_MOTOR_CANID = 6;
    public static final int FRONT_LEFT_CANCODER = 10;
    public static final String FRONT_LEFT_ENCODER_OFFSET_KEY = "2024/frontLeftEncoderOffset";

    public static final int BACK_LEFT_DRIVE_MOTOR_CANID = 3;
    public static final int BACK_LEFT_ROTATION_MOTOR_CANID = 7;
    public static final int BACK_LEFT_CANCODER = 11;
    public static final String BACK_LEFT_ENCODER_OFFSET_KEY = "2024/backLeftEncoderOffset";

    public static final int BACK_RIGHT_DRIVE_MOTOR_CANID = 4;
    public static final int BACK_RIGHT_ROTATION_MOTOR_CANID = 8;
    public static final int BACK_RIGHT_CANCODER = 12;
    public static final String BACK_RIGHT_ENCODER_OFFSET_KEY = "2024/backRightEncoderOffset";

    
  }

  public static class FieldConstants {
    public static final double ROBOT_WIDTH = 0.552; // in meters, with bumpers? find out
    public static final double ROBOT_LENGTH = 0.552; // in meters, with bumpers? find out
  }

  public static class DriveCommandConstants {
    public static final double CONTROLLER_DEADBAND = 0.1;
    public static final double LEFT_STICK_SCALE = 2.5;
    public static final double RIGHT_STICK_SCALE = 5;

  }
}
