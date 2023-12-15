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
    public static final int BR = 42;
  }

  public static final class DriveConstants {
    public static final double kTurnP = 0.015;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0.001;

    public static final double kTurnToleranceDeg = 0.5;
    public static final double kTurnRateToleranceDegPerS =5;

    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    
    public static final double kDriveToleranceMeters = 0.4;
    public static final double kDriveRateToleranceMetersPerS =0.05;

    public static final double maxDriveVelocity = 1;
    public static final double maxDriveAcceleration = 5;

    public static final double maxTurnVelocity = 250;
    public static final double maxTurnAcceleration = 500;
    //public static final double encoderTicsPerFoot = 6.84;


    //0.00075 bad too slow
    //
  }

  
}

