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
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 26;
    public static final int LEFT_FOLLOWER_ID = 8;
    public static final int RIGHT_LEADER_ID = 31;
    public static final int RIGHT_FOLLOWER_ID = 34;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    public static final double MOTOR_SPEED = 0.50;

    public static final boolean DRIVE_TYPE = true; // True = Diff. False = Tank
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 29;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_EJECT_VALUE = 0.44;

    public static final double ROLLER_OUT_SPEED = 0.55; // AWAY FROM POLYCARB
    public static final double ROLLER_IN_SPEED = 0.60; // TOWARD POLYCARB
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
}
