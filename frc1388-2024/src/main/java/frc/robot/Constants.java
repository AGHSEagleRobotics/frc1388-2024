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
  }

  public static class FieldConstants {
    // XXX I think this is right, but if the robot drives funny, check these numbers
  public static final double ROBOT_WIDTH = 0.508; // in meters, with bumpers? find out
  public static final double ROBOT_LENGTH = 0.508; // in meters, with bumpers? find out
  }

  public static class LimelightConstants {
    
  }

  public static class AutoConstants
  {
    public static final double TURN_P_VALUE = 0.062;
    public static final double TURN_P_TOLERANCE = 1.25;
    public static final double TURN_I_VALUE = 0.005;
    public static final double TURN_D_VALUE = 0.002;
    public static final double MOVE_P_VALUE = 0.045;
    public static final double MOVE_P_TOLERANCE = 0.5;

    public static final double CURVE_P_VALUE = 0.025;
    public static final double CURVE_MAX = 0.25;

    public static final double TURN_MIN_SPEED_STOPPED = 0.12;
    public static final double TURN_MIN_SPEED_MOVING = 0.4;
    public static final double TURN_MIN_SPEED_THRESHOLD = 2;
  }
}
