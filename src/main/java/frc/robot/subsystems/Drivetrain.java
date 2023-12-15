// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Drivetrain extends SubsystemBase implements Loggable {
  private final CANSparkMax motorFR = new CANSparkMax(Constants.CAN.FR, MotorType.kBrushless);
  private final CANSparkMax motorFL = new CANSparkMax(Constants.CAN.FL, MotorType.kBrushless);
  private final CANSparkMax motorBR = new CANSparkMax(Constants.CAN.BR, MotorType.kBrushless);
  private final CANSparkMax motorBL = new CANSparkMax(Constants.CAN.BL, MotorType.kBrushless);

  private final RelativeEncoder FLEncoder = motorFL.getEncoder();
  private final RelativeEncoder FREncoder = motorFR.getEncoder();
  private final RelativeEncoder BLEncoder = motorBL.getEncoder();
  private final RelativeEncoder BREncoder = motorBR.getEncoder();
  
  private final DifferentialDrive robotDrive = new DifferentialDrive(motorFL, motorFR);

  private final AHRS gyro = new AHRS();

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


  // DriveToDistance controller inputs
  // Uses constants as initial values
  private double driveP = Constants.DriveConstants.kDriveP;
  private double driveI = Constants.DriveConstants.kDriveI;
  private double driveD = Constants.DriveConstants.kDriveD;
  private double maxDriveVelocity = Constants.DriveConstants.maxDriveVelocity;
  private double maxDriveAcceleration = Constants.DriveConstants.maxDriveAcceleration;
  
  public Drivetrain() {
    motorFL.setInverted(true);
    motorFR.setInverted(false);
    motorBR.follow(motorFR);
    motorBL.follow(motorFL);

    motorFR.setIdleMode(IdleMode.kBrake);
    motorFL.setIdleMode(IdleMode.kBrake);
    motorBR.setIdleMode(IdleMode.kBrake);
    motorBL.setIdleMode(IdleMode.kBrake);

    
    FLEncoder.setPositionConversionFactor(0.4788/10.75);
    FREncoder.setPositionConversionFactor(0.4788/10.75);
    //meters per rotation, gear ratio
    
    FLEncoder.setVelocityConversionFactor(0.4788/10.75/60);
    FREncoder.setVelocityConversionFactor(0.4788/10.75/60);
    //meters per wheel rotation, gearing reduction, divide by 60 for per second

    motorFR.burnFlash();
    motorFL.burnFlash();
    motorBR.burnFlash();
    motorBL.burnFlash();
  }

  // Getters
  @Log
  public double getDistance() {
    return motorFR.getEncoder().getPosition();
  }

  @Log
  public double getYaw() {
    return gyro.getYaw();
  }

  @Log
  public double getDriveP() {
    return driveP;
  }

  @Log
  public double getDriveI() {
    return driveI;
  }

  @Log
  public double getDriveD() {
    return driveD;
  }

  @Log
  public double getMaxDriveAccel() {
    return maxDriveAcceleration;
  }

  @Log
  public double getMaxDriveVel() {
    return maxDriveVelocity;
  }

  // Setters
  public void resetGyro() {
    gyro.reset();
  }

  @Config
  public void setDrivePID(double p, double i, double d) {
    driveP = p;
    driveI = i;
    driveD = d;
  }

  @Config
  public void setMaxDriveVelocity(double velocity) {
    maxDriveVelocity = velocity;
  }

  @Config
  public void setMaxDriveAccel(double accel) {
    maxDriveAcceleration = accel;
  }

  public void autonDrive(double speed, double turn) {
    
    // to compensate for the turning/binding on one side, add x to turn
    // turn = turn + x;

    this.speed = speed;
    this.turn = turn;

    // robotDrive.arcadeDrive(accelLimit.calculate(speed), turnLimit.calculate(turn), false);
    robotDrive.arcadeDrive(speed, turn, false);

    robotDrive.feed();
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