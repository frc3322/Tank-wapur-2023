// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends ProfiledPIDCommand{
  /** Creates a new TurnToAngle. */

  private Drivetrain drivetrain;

  public TurnToAngle(double turnTarget, Drivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Constants.DriveConstants.kTurnP,
            Constants.DriveConstants.kTurnI,
            Constants.DriveConstants.kTurnD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.DriveConstants.maxTurnVelocity, Constants.DriveConstants.maxTurnAcceleration)),
        // This should return the measurement
        drivetrain::getYaw,
        // This should return the goal (can also be a constant)
        turnTarget,
        // This uses the output
        (output, setpoint) -> drivetrain.autonDrive(0, output),

        drivetrain
      );

    this.drivetrain = drivetrain;

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController()
        .setTolerance(Constants.DriveConstants.kTurnToleranceDeg, Constants.DriveConstants.kTurnRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }

  @Override
  public void initialize() {
    // End when the controller is at the reference.
    drivetrain.resetGyro();
    drivetrain.autonDrive(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.autonDrive(0, 0);
  }
}
