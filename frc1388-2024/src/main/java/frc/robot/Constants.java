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
    public static final int kOperatorControllerPort = 1;
  }

  public static class Swerve {
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

  public static class ShooterCommandConstants {
    public static final double kInitialMotorSpeed = 0.3;
    public static final double kShooterTimer = 10;

  }

  public static class ShooterSubsystemConstants {
    public static final double kShooterP = 0.00025;
    public static final double kShooterI = 0; //change later as needed
    public static final double kShooterD = 0;
    public static final double kShooterFF = 0;

    public static final int kShooterMotor1CANID = 13;
    public static final int kShooterMotor2CANID = 14;

  }

  public static class TransitionSubsystemConstants {
    public static final int MOTOR_CANID = 21;
  }

  public static class IntakeConstants {
    public static final int ROLLER_MOTOR_CANID = 19;
    public static final int LIFTER_MOTOR_CANID = 20;
    public static final int LOWER_LIMIT_DIO = 3;
    public static final int UPPER_LIMIT_DIO = 2;
    public static final int BEAM_BREAK_DIO = 1;
    public static final double ROLLER_MOTOR_SPEED = 0.7;
    public static final double LIFTER_MOTOR_SPEED_DOWN = -0.1; //XXX -0.2 for final desing
    public static final double LIFTER_MOTOR_SPEED_UP = 0.2; //XXX 0.4 for final design
  }

  public static class FieldConstants {
    public static final double ROBOT_WIDTH = 0.552; // in meters, with bumpers? find out
    public static final double ROBOT_LENGTH = 0.552; // in meters, with bumpers? find out
  }

  public static class LimelightConstants {
    
  }

  public static class AutoConstants
  {
    public static final double TURN_P_VALUE = 0.003;
    public static final double TURN_P_TOLERANCE = 1.25;
    public static final double TURN_I_VALUE = 0.003;
    public static final double TURN_D_VALUE = 0.004;
    
    public static final double MOVE_P_VALUE = 0.045;
    public static final double MOVE_P_TOLERANCE = 0.5;

    public static final double CURVE_P_VALUE = 0.025;
    public static final double CURVE_MAX = 0.25;

    public static final double TURN_MIN_SPEED_STOPPED = 0.12;
    public static final double TURN_MIN_SPEED_MOVING = 0.4;
    public static final double TURN_MIN_SPEED_THRESHOLD = 2;

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

  public static class DriveCommandConstants {
    public static final double CONTROLLER_DEADBAND = 0.1;
    public static final double LEFT_STICK_SCALE = 2.5;
    public static final double RIGHT_STICK_SCALE = 5;

    
  }
}

