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

  private final CANSparkMax bottomMotor = new CANSparkMax(Constants.CAN.ballBottomMotor, MotorType.kBrushless);
  private final CANSparkMax topMotor = new CANSparkMax(Constants.CAN.ballTopMotor, MotorType.kBrushless);

  public YogaBallLauncher() {
    bottomMotor.setIdleMode(IdleMode.kBrake);
    topMotor.setIdleMode(IdleMode.kBrake);

    bottomMotor.burnFlash();
    topMotor.burnFlash();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinIntake(double volts){
    bottomMotor.setVoltage(volts);
    topMotor.setVoltage(volts*3);
  }
}
