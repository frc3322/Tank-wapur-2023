// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax motorFR = new CANSparkMax(Constants.CAN.FR, MotorType.kBrushless);
  private final CANSparkMax motorFL = new CANSparkMax(Constants.CAN.FL, MotorType.kBrushless);
  private final CANSparkMax motorBR = new CANSparkMax(Constants.CAN.BR, MotorType.kBrushless);
  private final CANSparkMax motorBL = new CANSparkMax(Constants.CAN.BL, MotorType.kBrushless);

  private final RelativeEncoder FLEncoder = motorFL.getEncoder();
  private final RelativeEncoder FREncoder = motorFR.getEncoder();
  private final RelativeEncoder BLEncoder = motorBL.getEncoder();
  private final RelativeEncoder BREncoder = motorBR.getEncoder();
  
  private final DifferentialDrive robotDrive = new DifferentialDrive(motorFL, motorFR);

  private double speed = -2;
  private double turn = -2;

  private final SlewRateLimiter accelLimit = new SlewRateLimiter(3);
  private final SlewRateLimiter turnLimit = new SlewRateLimiter(5);

  // //Variables to log voltage
  double FLVoltageVal;
  double FRVoltageVal;
  double BLVoltageVal;
  double BRVoltageVal;

  
  //  //Variables to log voltage
   double FRVelocityVal;
   double FLVelocityVal;
   double BLVelocityVal;
   double BRVelocityVal;
  /** Creates a new Drivetrain. */
  
  public Drivetrain() {
    motorFL.setInverted(true);
    motorFR.setInverted(false);
    motorBR.follow(motorFR);
    motorBL.follow(motorFL);

    motorFR.setIdleMode(IdleMode.kBrake);
    motorFL.setIdleMode(IdleMode.kBrake);
    motorBR.setIdleMode(IdleMode.kBrake);
    motorBL.setIdleMode(IdleMode.kBrake);

    motorFR.burnFlash();
    motorFL.burnFlash();
    motorBR.burnFlash();
    motorBL.burnFlash();
  }
  public void resetEncoders() {
    motorFL.getEncoder().setPosition(0);
    motorFR.getEncoder().setPosition(0);
    motorBL.getEncoder().setPosition(0);
    motorBR.getEncoder().setPosition(0);
  }

  public void drive(double speed, double turn) {
    turn = 0.5 * turn + 0.5 * Math.pow(turn, 3);  // Weird math

    this.speed = speed;
    this.turn = turn;

   robotDrive.arcadeDrive(accelLimit.calculate(speed), turnLimit.calculate(turn), false);
    //robotDrive.arcadeDrive(speed, turn, false);

    robotDrive.feed();
  }
  public void tankDriveVolts(double leftVolts, double rightVolts){
    motorFL.setVoltage(leftVolts);
    motorFR.setVoltage(rightVolts);
    
    robotDrive.feed();
  }

    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    FLVoltageVal = motorFL.getBusVoltage();
    FRVoltageVal = motorFR.getBusVoltage();
    BLVoltageVal = motorBL.getBusVoltage();
    BRVoltageVal = motorBR.getBusVoltage();

    FLVelocityVal = FLEncoder.getVelocity();
    FRVelocityVal = FREncoder.getVelocity();
    BLVelocityVal = BLEncoder.getVelocity();
    BRVelocityVal = BREncoder.getVelocity();
  }
}