// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final class Drivetrain {
    public static int FL_DRIVE_ID = 27;
    public static int FL_STEER_ID = 34;
    public static int FR_DRIVE_ID = 15;
    public static int FR_STEER_ID = 45;
    public static int RL_DRIVE_ID = 5;
    public static int RL_STEER_ID = 49;
    public static int RR_DRIVE_ID = 6;
    public static int RR_STEER_ID = 50;

    /* Units per rotation for analog/absolute encoders on Talon SRX */
    public static double ANALOG_UPR = 1024;

    public static double WHEEL_DIAMETER = (4 * 2.54) / 100; // Convert to meters
    public static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static double DRIVE_GEAR_RATIO = 6.67/1;
    public static double STEER_GEAR_RATIO = 1;

    public static double STEER_F = 0.0;
    public static double STEER_P = 6.5;
    public static double STEER_I = 0.0;
    public static double STEER_D = 0.0;

    public static double DRIVE_KS = 0.5;
    public static double DRIVE_KV = 1.0;
    public static double DRIVE_KA = 0.0;

    public static Translation2d FL_POS = new Translation2d(1, 1);
    public static Translation2d FR_POS = new Translation2d(1, -1);
    public static Translation2d RL_POS = new Translation2d(-1, 1);
    public static Translation2d RR_POS = new Translation2d(-1, -1);

    public static Rotation2d FL_OFFSET = new Rotation2d(0); 
    public static Rotation2d FR_OFFSET = new Rotation2d(0);
    public static Rotation2d RL_OFFSET = new Rotation2d(0);
    public static Rotation2d RR_OFFSET = new Rotation2d(0);

    public static double MAX_LIN_VEL = 1;
    public static double MAX_ROT_VEL = 1;
  }

  public static final class Controllers {
    public static double JOYSTICK_DEADBAND = 0.15;
  }

}
