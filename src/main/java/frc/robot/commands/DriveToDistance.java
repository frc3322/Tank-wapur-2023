// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistance extends PIDCommand {

  private Drivetrain drivetrain;
  private double targetDistance;

  /** Creates a new DriveToDistance. */
  public DriveToDistance(double targetDistance, Drivetrain drivetrain) {
    super(
      new PIDController(Constants.DriveConstants.kDriveP, Constants.DriveConstants.kDriveI, Constants.DriveConstants.kDriveD),
        // Close loop on heading
        drivetrain::getDistance,
        // Set reference to target
        -targetDistance,
        // Pipe output to turn robot
        output -> drivetrain.autonDrive(output,0),
        // Require the drive
        drivetrain);

    this.drivetrain = drivetrain;
    this.targetDistance = targetDistance;
    
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.DriveConstants.kDriveToleranceMeters, Constants.DriveConstants.kDriveRateToleranceMetersPerS);
  }

  @Override
  public void initialize() {
    drivetrain.resetEncoders();
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
