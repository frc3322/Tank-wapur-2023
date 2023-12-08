// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistance extends ProfiledPIDCommand {

  private Drivetrain drivetrain;
  private double targetDistance;

  /** Creates a new DriveToDistance. */
  public DriveToDistance(double targetDistance, Drivetrain drivetrain) {
    super(
      new ProfiledPIDController(Constants.DriveConstants.kDriveP, Constants.DriveConstants.kDriveI, Constants.DriveConstants.kDriveD, new TrapezoidProfile.Constraints(Constants.DriveConstants.maxDriveVelocity, Constants.DriveConstants.maxDriveAcceleration)),
        // Close loop on heading
        drivetrain::getDistance,
        // Set reference to target
        -targetDistance,
        // Pipe output to turn robot
        (output, setpoint) -> drivetrain.autonDrive(output,0),
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.autonDrive(0, 0);;
  }
}
