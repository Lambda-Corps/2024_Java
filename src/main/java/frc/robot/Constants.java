// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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

    // Robot Physical Characteristics with dimensional units
    public static double ROBOT_MASS = 90;
    public static double DT_GEAR_RATIO = 6.0;
    public static double DT_MOTORS_PER_SIDE = 2;
    public static double DT_WHEEL_DIAMETER = 6;
    public static double DT_WHEEL_RADIUS_INCHES = DT_WHEEL_DIAMETER / 2;
    public static double DT_WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(DT_WHEEL_DIAMETER * Math.PI);
    public static double DT_TRACKWIDTH_METERS = 0.546;
    public static double DT_TICKS_PER_MOTOR_REV = 2048;
    public static double DT_TICKS_PER_INCH = (DT_TICKS_PER_MOTOR_REV * DT_GEAR_RATIO) / ((2 * Math.PI) * DT_WHEEL_DIAMETER);
    public static double DT_TICKS_PER_METER = (DT_TICKS_PER_MOTOR_REV * DT_GEAR_RATIO) / ((2 * Math.PI) * DT_WHEEL_DIAMETER);
    public static double ROBOT_WHEELBASE = 22;
    public static double ROBOT_BUMPER_WIDTH = 3.25;
    public static double ROBOT_WIDTH = 23 + (ROBOT_BUMPER_WIDTH * 2);
    public static double ROBOT_LENGTH = 32 + (ROBOT_BUMPER_WIDTH * 2);


    //CAN IDS
    public static int DT_LEFT_LEADER = 1;  
    public static int DT_RIGHT_LEADER = 2;
    public static int DT_LEFT_FOLLOWER = 3;
    public static int DT_RIGHT_FOLLOWER = 4;
    public static int INTAKE_MOTOR = 5;
    // 6
    // 7
    // 8
    // 9
    // 10
    // 11
    // 12
    // 13
    // 14

    // Default Robot loop period
    public static double ROBOT_PERIOD_MS = 0.020;  // 50Hz, or 20 times a second

    // Controller Mapping information
    public static int DRIVER_CONTROLLER = 0;
    public static int PARTNER_CONTROLLER = 1;
    public static int CONTROLLER_FORWARD_REAL = 1;
    public static int CONTROLLER_FORWARD_SIM = 1;
    public static int CONTROLLER_TURN_REAL = 4;
    public static int CONTROLLER_TURN_SIM = 0;

}
