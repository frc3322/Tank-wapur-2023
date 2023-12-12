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
    public static final int FL = 13;
    public static final int FR = 5;
    public static final int BL = 36;
    public static final int BR = 42; // NOT FINAL - UPDATE WHEN AVAILABLE -

    // Shelf motor can id
    public static final int shelfMotorLeft = 37;
    public static final int shelfMotorRight = 40;

    // Forklift motor
    public static final int forkLiftMotor = 100;
    public static final int forkLiftVolts = 3;

    // Timeouts for shelf and forklift
    public static final int shelfTimeout = 9;
    public static final int forkLiftTimeout = 5;
    
    // yoga ball pickup/launcher
    public static final int ballBottomMotor = 4;
    public static final int ballTopMotor = 8;
  }

  // Drivetrain constants
  public static final int SlewAccelLimit = 15;
  public static final int SlewTurnLimit = 24;

  public static final class shelfConstants {
    // Motor speed of the shelf extension motor, negated when retracting
    public static final int shelfMotorSpeed = 2;

    // Limit in encoder whatever units untill the pusher is fully extended
    public static final int shelfExtensionLimit = 50;
  }

  public static final class yogaBallConstants {
    // volts to use when picking up or dropping the ball
    public static final int yogaBallVolts = 2;
  }
}

