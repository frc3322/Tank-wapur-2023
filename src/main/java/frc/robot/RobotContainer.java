// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveToDistance;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  SendableChooser<Command> autChooser = new SendableChooser<>();

  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Command driveCommand = new RunCommand(
      () -> {
        double speed = MathUtil.applyDeadband(driverController.getLeftY(), 0.09);
        double turn = MathUtil.applyDeadband(driverController.getRightX(), 0.08);
       drivetrain.drive(speed, turn);
      }, drivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    autChooser.addOption("drive distance test", new TestDriveDist());

    autChooser.addOption("turn to angle test", new TestTurnToAngle());

    autChooser.addOption("Test Auton", new TestAuton());
    
    SmartDashboard.putData("select autonomous", autChooser);

    // The first argument is the root container
    // The second argument is whether logging and config should be given separate tabs
    Logger.configureLoggingAndConfig(this, false);
  }

  public void updateLogger() {
    Logger.updateEntries();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
        drivetrain.setDefaultCommand(driveCommand);
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autChooser.getSelected();
  }

  private class TestDriveDist extends SequentialCommandGroup{
    private TestDriveDist(){
        addCommands(
            new InstantCommand(
              () -> drivetrain.resetEncoders(),
               drivetrain
            ),
            new DriveToDistance(5, drivetrain)
        );
    }
  }

  private class TestTurnToAngle extends SequentialCommandGroup{
    private TestTurnToAngle(){
      addCommands(
        new TurnToAngle(-90, drivetrain)
      );
    }
  }

  private class TestAuton extends SequentialCommandGroup{
    private TestAuton(){
      addCommands(
        new InstantCommand(
          () -> drivetrain.resetEncoders(),
            drivetrain
        ),
        new DriveToDistance(5, drivetrain));

      addCommands(
      new TurnToAngle(180, drivetrain));
    }
  }
}
