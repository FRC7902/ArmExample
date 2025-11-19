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
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
  public static class ArmConstants {
    public static final int ARM_MOTOR_CAN_ID = 1;

    public static final double ARM_LENGTH_METERS = 0.3;
    public static final double ARM_MASS_KG = 3.5;
    public static final double ARM_GEARING = 67.5;

    public static final double ARM_MAX_ANGLE_DEGREES = 180.0;
    public static final double ARM_MIN_ANGLE_DEGREES = 0.0;

    public static final double ARM_KP = 50;
    public static final double ARM_KI = 0;
    public static final double ARM_KD = 0;

    public static final double ARM_TARGET_TOLERANCE = 3.0;

    public static final double STATOR_CURRENT_LIMIT = 50.0;
    public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  }
}
