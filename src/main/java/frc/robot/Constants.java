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
  public static final class CAN {
    // Drivetrain motors
    public static final int FL = 6;
    public static final int FR = 5;
    public static final int BL = 36;
    public static final int BR = 0; // NOT FINAL - UPDATE WHEN AVAILABLE -

    // Shelf motor can id
    public static final int shelfMotorLeft = 100;
    public static final int shelfMotorRight = 101;


    // yoga ball pickup/launcher
    public static final int ballLeftMotor = 111;
    public static final int ballRightMotor = 222;
  }

  public static final class shelfConstants {
    // Motor speed of the shelf extension motor, negated when retracting
    public static final int shelfMotorSpeed = 5;

    // Limit in encoder whatever units untill the pusher is fully extended
    public static final int shelfExtensionLimit = 10;
  }

  public static final class yogaBallConstants {
    // volts to use when picking up or dropping the ball
    public static final int yogaBallVolts = 5;
  }
}

