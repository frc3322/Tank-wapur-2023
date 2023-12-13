// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// This is the new branch test
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class Lifter extends SubsystemBase {
  private final CANSparkMax forkLiftMotor = new CANSparkMax(Constants.CAN.forkLiftMotor, MotorType.kBrushless);
  /** Creates a new Lifter. */
  public Lifter() {
    forkLiftMotor.setIdleMode(IdleMode.kBrake);
    forkLiftMotor.burnFlash();
  }

  public void lift(double volts) {
    forkLiftMotor.setVoltage(volts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
