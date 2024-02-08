// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static class SwerveModuleConstants { 
    public static final double DIST_PER_TICK = (1.0 / 6.75) * (0.3192); // ask calvin about the math

    public static final double kDriveMotorP = 0.001;
    public static final double kDriveMotorI = 0;
    public static final double kDriveMotorD = 0;

    public static final double kRotationP = 0.007;
    public static final double kRotationI = 0;
    public static final double kRotationD = 0;

    public static final double kRotationTolerance = 5;
  }

  public static class DriveTrainConstants {
    public static final double ROBOT_MAX_SPEED = 3.0; //meters per second

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
    // XXX I think this is right, but if the robot drives funny, check these numbers
  public static final double ROBOT_WIDTH = 0.508; // in meters, with bumpers? find out
  public static final double ROBOT_LENGTH = 0.508; // in meters, with bumpers? find out
  }

  public static class DriveCommandConstants {
    public static final double CONTROLLER_DEADBAND = 0.1;
    public static final double LEFT_STICK_SCALE = 2.5;
    public static final double RIGHT_STICK_SCALE = 5;

  }

  public static class AutoConstants {
    // where all the constants used for Auto will be

      public enum Objective{
        SITSTILL ("LookPretty"),
        LEAVEZONE ("LeaveZone");

      public static final Objective Default = SITSTILL;

      private String m_dashboardDescript; //This is what will show on dashboard
      private Objective ( String dashboardDescript ) {
        m_dashboardDescript = dashboardDescript;
      }

      public String getDashboardDescript() {
        return m_dashboardDescript;
      }  
      }

      public enum Position {
        CLOSE("CLOSE"), //need to rename this
        MID("MID"),
        FAR("FAR");
    
        public static final Position Default = CLOSE;
    
        private String m_dashboardDescript; //This is what will show on dashboard
        private Position ( String dashboardDescript ) {
          m_dashboardDescript = dashboardDescript;
        }
    
        public String getDashboardDescript() {
          return m_dashboardDescript;
        }
      }

  }
}
