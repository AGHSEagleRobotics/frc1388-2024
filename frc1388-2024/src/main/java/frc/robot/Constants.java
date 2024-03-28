// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class SwerveModuleConstants {
    public static final double DIST_PER_TICK = (1.0 / 6.75) * (0.3192); // ask calvin about the math

    public static final double DIST_PER_MOTOR_ROTATION  = 5.65 / 131.14; // calvins magic numbers

    public static final double DRIVE_MOTOR_P = 0.001;
    public static final double DRIVE_MOTOR_I = 0;
    public static final double DRIVE_MOTOR_D = 0;

    public static final double ROTATION_MOTOR_P = 0.007;
    public static final double ROTATION_MOTOR_I = 0;
    public static final double ROTATION_MOTOR_D = 0;

    public static final double ROTATION_TOLERANCE = 5;
  }

  public static class DriveTrainConstants {
    public static final double ROBOT_MAX_SPEED = 4.0; // meters per second

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

    public static final double CONTROLLER_DEADBAND = 0.1;
    public static final double MANUAL_CONTROL_ANGLE_DEADBAND = 0.5;
    public static final double LEFT_STICK_SCALE = 2.5;
    public static final double RIGHT_STICK_SCALE = 5;
  }

  // XXX organise these
  public static class ShooterConstants {
    public static final int BOTTOM_SHOOTER_MOTOR_CANID = 13;
    public static final int TOP_SHOOTER_MOTOR_CANID = 14;

    public static final double SHOOTER_MOTOR_P = 0.001; // was 0.00025;
    public static final double SHOOTER_MOTOR_I = 0; // change later as needed
    public static final double SHOOTER_MOTOR_D = 0;
    public static final double SHOOTER_MOTOR_FF = 0;

    public static final double SPEAKER_SHOT_RPM = 4000;
    public static final double AMP_SHOT_RPM = 450;
    public static final double SHOOTER_TIMER = 10;
    public static final int kShooterMotor1CANID = 0;
    public static final int kShooterMotor2CANID = 0;

  }

  public static class TransitionConstants {
    public static final int TRANSITION_MOTOR_CANID = 21;

    /** feeding into the shooter */
    public static final double TRANSITION_MOTOR_POWER_IN = 1.0;
    /** ejecting in direction opposite of the shooter. */
    public static final double TRANSITION_MOTOR_POWER_OUT = -1.0;
  }

  public static class ShooterAngleSubsystemConstants {
    public static final double kShooterAngleP = 45;
    public static final double kShooterAngleD = 0;
    public static final double kShooterAngleFF = 0;
    public static final int kPotentiometerAnalogIN = 3;

    public static final double kShooterPositionSpeaker = 0.325; // previously 0.37 for competition matches
    public static final double kShooterPositionDrewSaucyShot = 0.258; 
    public static final double kShooterPositionPodium = 0.173; 
    public static final double kShooterLowClamp = 0.168;
    public static final double kShooterPositionStartLine = 0.277;
    public static final double kShooterPositionNoteB = 0.225; // previously 0.266 for practice matches
    public static final double kShooterPositionMax = 0.4; // observed max is roughly
    public static final double kShooterPositionMin = 0; //  observed min without hitting transition 

    public static final int kShooterAngleMotorCANID = 22;

    public static final double P_TOLERANCE = 0.003;
  }

  public static class IntakeConstants {
    public static final int ROLLER_MOTOR_CANID = 19;
    public static final int LIFTER_MOTOR_CANID = 20;
    public static final int LOWER_LIMIT_DIO = 3;
    public static final int UPPER_LIMIT_DIO = 2;
    public static final int BEAM_BREAK_DIO = 1;

    public static final double ROLLER_MOTOR_SPEED_IN_INTAKING = 0.7;
    public static final double ROLLER_MOTOR_SPEED_IN_TRANSITION = 0.4;
    public static final double ROLLER_MOTOR_SPEED_OUT = -0.7;

    public static final double LIFTER_MOTOR_TIME_DOWN = 0.6;
    public static final double LIFTER_MOTOR_TIME_OFF = LIFTER_MOTOR_TIME_DOWN + 0.1;
    public static final double LIFTER_MOTOR_TIME_COAST = LIFTER_MOTOR_TIME_DOWN + 0.8;

    public static final double LIFTER_MOTOR_SPEED_DOWN = -0.4;
    public static final double LIFTER_MOTOR_SPEED_UP = 0.5;
    public static final double LIFTER_MOTOR_SPEED_UP_HOLD = 0.2;

    /** The amount of consecutive tick the beambreak must be triggered for the intake to retract. */
    public static final int TICKS_BEFORE_RETRACTING_INTAKE = 4;
  }

  public static class FieldConstants {
    public static final double ROBOT_WIDTH = 0.552; // in meters, with bumpers? find out
    public static final double ROBOT_LENGTH = 0.552; // in meters, with bumpers? find out
    public static final double SUBLIFER_LENGTH = 0.91;
  }

  public static class LimelightConstants {
    public static final double MAX_TY_VALUE = 8.5;
    public static final double SLOW_DOWN = 0.5;
    public static final double METERS_PER_SECOND = 0.2;
    public static final double TURN_P_VALUE_AUTO_TRACKING = 0.14;
    public static final double TURN_I_VALUE_AUTO_TRACKING = 0.000012;
    public static final double TURN_D_VALUE_AUTO_TRACKING = 0.000012;
    // public static final double TURN_P_VALUE_AUTO_TRACKING = 0.044;
    // public static final double TURN_I_VALUE_AUTO_TRACKING = 0.000024;
    // public static final double TURN_D_VALUE_AUTO_TRACKING = 0.000024;
    public static final double TX_OFFSET = 0;
    public static final double DISTANCE_FROM_APRILTAG_PODIUM = 2.95;
    public static final double DISTANCE_FROM_APRILTAG_AUTOSHOOTER = 3.2;
    public static final double DISTANCE_FROM_APRILTAG_SUBWOOFER = 1.35;
    public static final double DISTANCE_FROM_APRILTAG_POSITIONB = 2.54;
    public static final double DISTANCE_FROM_APRILTAG_STARTLINE = 2.40;
    public static final double DISTANCE_FROM_APRILTAG_WING = 5.87248;
    public static final double LED_USE_PIPELINE = 0;
    public static final double LED_FORCE_OFF = 1;
    public static final double LED_FORCE_BLINK = 2;
    public static final double LED_FORCE_ON = 3;

    public static final int BOTPOSE_X = 0;
    public static final int BOTPOSE_Y = 1;
    public static final int BOTPOSE_Z = 2;
    public static final int BOTPOSE_ROLL = 3;
    public static final int BOTPOSE_PITCH = 4;
    public static final int BOTPOSE_YAW = 5;
    public static final int BOTPOSE_LATENCY = 6;
    public static final int BOTPOSE_TOTAL_APRILTAGS_SEEN = 7;
    public static final int BOTPOSE_DISTANCE_BETWEEN_TAGS = 8;
    public static final int BOTPOSE_AVERAGE_DISTANCE_FROM_CAMERA = 9;
    public static final int BOTPOSE_AVERAGE_TAG_AREA = 10;
    public static final int BOTPOSE_TAG_ID = 11;
    public static final int BOTPOSE_TX_RAW_TARGET_ANGLE = 12;
    public static final int BOTPOSE_TY_RAW_TARGET_ANGLE = 13;
    public static final int BOTPOSE_TARGET_AREA = 14;
    public static final int BOTPOSE_DISTANCE_TO_CAMERA = 15;
    public static final int BOTPOSE_DISTANCE_TO_ROBOT = 16;
    public static final int BOTPOSE_AMBIGUITY = 17;

    public static final int BOTPOSE_NEXT_INDEX_OF_TAG_ID = 7;
    public static final int BOTPOSE_RELATIVE_TAG_ID = 0;
    public static final int BOTPOSE_RELATIVE_TX_RAW_TARGET_ANGLE = 1;
    public static final int BOTPOSE_RELATIVE_TY_RAW_TARGET_ANGLE = 2;
    public static final int BOTPOSE_RELATIVE_TARGET_AREA = 3;
    public static final int BOTPOSE_RELATIVE_DISTANCE_TO_CAMERA = 4;
    public static final int BOTPOSE_RELATIVE_DISTANCE_TO_ROBOT = 5;
    public static final int BOTPOSE_RELATIVE_AMBIGUITY = 6;

    public static final double DISTANCE_OFFSET_CAMERA = 0.15;

    public static final double ID_7_AND_4_LOCATION_Y = 5.547;
    public static final double ID_7_LOCATION_X_BLUE = -0.0381;
    public static final double ID_4_LOCATION_X_RED = 16.579;

    // public static final double SLOPE_MATH_SUBLIFER_TO_POSITIONB = (ShooterAngleSubsystemConstants.kShooterPositionNoteB - 
    // ShooterAngleSubsystemConstants.kShooterPositionSpeaker) / (DISTANCE_FROM_APRILTAG_POSITIONB - DISTANCE_FROM_APRILTAG_SUBWOOFER);
    // public static final double SHOOTER_OFFSET_SUBTOB = 0.5177895;
    // public static final double SLOPE_MATH_POSITIONB_TO_PODIUM = (ShooterAngleSubsystemConstants.kShooterPositionPodium - 
    // ShooterAngleSubsystemConstants.kShooterPositionNoteB) / (DISTANCE_FROM_APRILTAG_PODIUM - DISTANCE_FROM_APRILTAG_POSITIONB);
    // public static final double SHOOTER_OFFSET_B_TO_POD = 0.1149024;
    // public static final double SLOPE_MATH_PODIUM_TO_WING = (ShooterAngleSubsystemConstants.kShooterPositionWing - 
    // ShooterAngleSubsystemConstants.kShooterPositionPodium) / (DISTANCE_FROM_APRILTAG_WING - DISTANCE_FROM_APRILTAG_PODIUM);
    // public static final double SHOOTER_OFFSET_WING = 0.3305;

    // mycurvefit numbers for quadratic interpolation based on X Distance values and Y Shooter angles
    // values = 
    // 1.35, 0.325
    // 1.76 0.27
    // 2.18 0.232
    // 2.45 .21
    // 2.81 .177
    // 2.96 .173

    // new values based on bose estimation with trig
    // 1(limelight distance to speaker) 2(potentiomator position) 3(measured distance to speaker)
    // 1.41 0.325, 1.32
    // 1.73 0.27, 1.74
    // 2.16 0.22, 2.175
    // 2.43 0.200, 2.44
    // 2.79 0.178, 2.81
    // 2.91 0.169, 2.93
    public static final double QUADRATIC_AUTO_SHOOTER_A = 0.565951; 
    public static final double QUADRATIC_AUTO_SHOOTER_B = 0.01572077; 
    public static final double QUADRATIC_AUTO_SHOOTER_C = -0.286937;
    public static final double QUADRATIC_AUTO_SHOOTER_D = 0.1368955;
    public static final double QUADRATIC_AUTO_SHOOTER_E = -0.01933188;
  }

  public static class LEDConstants {
    public static final double RAINBOW = -0.99;
    public static final double RED_STROBE = -0.11;
    public static final double BLUE_STROBE = -0.09;
    public static final double RED_SOLID = 0.61;
    public static final double BLUE_SOLID = 0.87;
  }

  public static class AutoConstants {
    public static final double TURN_P_VALUE = 0.04;

    public static final double TURN_P_TOLERANCE = 2.5;
    public static final double TURN_I_VALUE = 0.003;
    public static final double TURN_D_VALUE = 0.004;

    public static final double MOVE_P_VALUE = 0.045;
    public static final double MOVE_P_TOLERANCE = 0.5;

    public static final double CURVE_P_VALUE = 0.025;
    public static final double CURVE_MAX = 0.25;

    public static final double TURN_MIN_SPEED_STOPPED = 0.12;
    public static final double TURN_MIN_SPEED_MOVING = 0.4;
    public static final double TURN_MIN_SPEED_THRESHOLD = 2;

    public static final double LEAVE_ZONE_FROM_SUB_DIST = -1.0;
    public static final double LEAVE_FROM_SIDE_LONG = -4.5;

    public enum Objective{
        SITSTILL ("LookPretty"),
        START1LEAVE ("1, Leave"),
        LEAVEANDSHOOT ("1, Shoot, Leave"),
        Shoot1IntakeBSpeakerB ("1, B"),
        Shoot123 ("1 Shoot"),
        ShootAndLeaveFromSideWithLonger("2 and leave"),
        FourNote ("4 note auto "),
        ThreeNote ("3 note auto "),
        testCoordinate("TEST DELETE ME");
//        Shoot3Leave ("3 Leave"); // needs testing
//        Shoot1IntakeBSpeakerBIntakeASpeakerA ("1, B, C"),
//        testCoordinate ("testcoordinate");

      public static final Objective Default = SITSTILL;

      private String m_dashboardDescript; // This is what will show on dashboard

      private Objective(String dashboardDescript) {
        m_dashboardDescript = dashboardDescript;
      }

      public String getDashboardDescript() {
        return m_dashboardDescript;
      }
    }
  }
}