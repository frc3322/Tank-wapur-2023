// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class YogaBallLauncher extends SubsystemBase {
  /** Creates a new YogaBallLauncher. */

  private final CANSparkMax leftMotor = new CANSparkMax(Constants.CAN.ballLeftMotor, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(Constants.CAN.ballRightMotor, MotorType.kBrushless);

  public YogaBallLauncher() {
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinIntake(double volts){
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(-volts);
  }
}
