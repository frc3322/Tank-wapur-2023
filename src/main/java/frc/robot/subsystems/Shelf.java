// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shelf extends SubsystemBase {
  /** Creates a new Shelf. */
  private final CANSparkMax axleMotor;

  private final RelativeEncoder shelfEncoder;

  private final boolean reverse;

  public Shelf(int CANid, boolean reverse) {
    axleMotor = new CANSparkMax(CANid, MotorType.kBrushless);

    shelfEncoder = axleMotor.getEncoder();

    axleMotor.setIdleMode(IdleMode.kBrake);

    axleMotor.burnFlash();

    this.reverse = reverse;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Boolean extended() {
    //return true if the encoder is past or at 1
    if (this.reverse) {
      return shelfEncoder.getPosition() >= -Constants.shelfConstants.shelfExtensionLimit;
    }
    return shelfEncoder.getPosition() >= Constants.shelfConstants.shelfExtensionLimit;
  }

  public Boolean retracted() {
    //return true if the encoder is at or less than 0 
    return shelfEncoder.getPosition() <= 0;
  }

  public void setMotorSpeed(double speed) {
    if (this.reverse){
      axleMotor.set(-speed);
    }
    axleMotor.set(speed);
  }

  public Command extendShelf() {
      //flips to bottom, does not spin. may be able to delete
      return new RunCommand(
        () -> setMotorSpeed(Constants.shelfConstants.shelfMotorSpeed)
      )
      .until(()->extended());
    }

  public Command retractShelf() {
    //flips to bottom, does not spin. may be able to delete
    return new RunCommand(
      () -> setMotorSpeed(-Constants.shelfConstants.shelfMotorSpeed)
    )
    .until(()->retracted());
  }

  public Command toggleShelf() {
    if (retracted()) {
      return extendShelf();
    }
    else {
      return retractShelf();
    }
  }
}
