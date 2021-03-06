/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.commands.DriveTeleOp;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.analog.adis16470.frc.ADIS16470_IMU;
import com.analog.adis16470.frc.ADIS16470_IMU.IMUAxis;
import com.analog.adis16470.frc.ADIS16470_IMU.ADIS16470CalibrationTime;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.controller.PIDController;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX frontLeftDrive;
  private final WPI_TalonSRX frontRightDrive;
  private final WPI_VictorSPX backLeftDrive;
  private final WPI_VictorSPX backRightDrive;
  //private final DifferentialDrive m_drive;

  //private final Encoder rightEncoder;
  //private final Encoder leftEncoder;

  private final int encoderCountsPerInch;

  private final ADIS16470_IMU imu;
  private final PIDController turn_PID;

  public double turn_kP;
  public double turn_kI;
  public double turn_kD;
  public double turn_tolerance;
  public double turn_derivativeTolerance;
  public double turn_error;
  

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    frontLeftDrive = new WPI_TalonSRX(Constants.frontLeftDrivePort);
    frontRightDrive = new WPI_TalonSRX(Constants.frontRightDrivePort);
    backLeftDrive = new WPI_VictorSPX(Constants.backLeftDrivePort);
    backRightDrive = new WPI_VictorSPX(Constants.backRightDrivePort);

    frontLeftDrive.configFactoryDefault();
    frontRightDrive.configFactoryDefault();
    backLeftDrive.configFactoryDefault();
    backRightDrive.configFactoryDefault();

    frontRightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    frontLeftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);

    frontLeftDrive.setInverted(true);
    frontRightDrive.setInverted(false);
    backLeftDrive.setInverted(true);
    backRightDrive.setInverted(false);

    frontLeftDrive.setSensorPhase(true);
    frontRightDrive.setSensorPhase(false);
    backLeftDrive.setSensorPhase(true);
    backRightDrive.setSensorPhase(false);

    backLeftDrive.follow(frontLeftDrive);
    backRightDrive.follow(frontRightDrive);
 
    //m_drive = new DifferentialDrive(frontRightDrive, backLeftDrive);

    encoderCountsPerInch = 0;

    
    imu = new ADIS16470_IMU(IMUAxis.kZ, SPI.Port.kOnboardCS0, ADIS16470CalibrationTime._4s);
    imu.reset();

    turn_kP = .3; turn_kI = 0; turn_kD = 0;
    turn_tolerance = 1;
    turn_derivativeTolerance = .01;
    turn_error = -imu.getRate();

    turn_PID = new PIDController(turn_kP, turn_kI, turn_kD);
    turn_PID.setTolerance(turn_tolerance, turn_derivativeTolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder", getDriveEncoderCount()[0]);
    SmartDashboard.putNumber("Right Encoder", getDriveEncoderCount()[1]);
  }

  public void driveByPercent(double leftSpeed, double rightSpeed)
  {
    frontLeftDrive.set(ControlMode.PercentOutput, leftSpeed);
    frontRightDrive.set(ControlMode.PercentOutput, rightSpeed);
    //backLeftDrive.set(ControlMode.Follower, frontLeftDrive.getDeviceID());
    //backRightDrive.set(ControlMode.Follower, frontRightDrive.getDeviceID());
  }

  //TODO: NEED FRC CHARACTERIZATION TOOL TO SET PID VALUES!!!
  public void driveByEncoderInches(int leftInches, int rightInches)
  {
    int leftCount = leftInches * encoderCountsPerInch;
    int rightCount = rightInches * encoderCountsPerInch;
    frontLeftDrive.set(ControlMode.Position, leftCount);
    frontRightDrive.set(ControlMode.Position, rightCount);
    //backLeftDrive.set(ControlMode.Follower, frontLeftDrive.getDeviceID());
    //backRightDrive.set(ControlMode.Follower, frontRightDrive.getDeviceID());
  }

  public void driveByAngle(double targetAngle)
  {
    double calculatedPID = turn_PID.calculate(getAngle(), targetAngle);
    frontLeftDrive.set(ControlMode.PercentOutput, calculatedPID);
    frontRightDrive.set(ControlMode.PercentOutput, -calculatedPID);
  }

  public double[] getDriveEncoderDistance(){
    return new double[] {frontLeftDrive.getSelectedSensorPosition() / encoderCountsPerInch, frontRightDrive.getSelectedSensorPosition() / encoderCountsPerInch};
  }

  public double[] getDriveEncoderCount(){
    return new double[] {frontLeftDrive.getSelectedSensorPosition(), frontRightDrive.getSelectedSensorPosition()};
  }

  public void resetEncoders()
  {
    frontLeftDrive.setSelectedSensorPosition(0, 0, 10);
    frontRightDrive.setSelectedSensorPosition(0, 0, 10);
  }

  public void resetGyro(){
    imu.calibrate();
  }

  public double getAngle(){
    return imu.getAngle();
  }

  public boolean atTargetAngle()
  {
    return turn_PID.atSetpoint();
  }

  public void resetTurnPID()
  {
    turn_PID.reset();
  }

  public void StraightDrive(double speed) {
    /*double gyroCurve = getAngle()/90.0;
	  if (speed > 0) {
      m_drive.curvatureDrive(speed, gyroCurve, false);
  	}
  	else {
      m_drive.curvatureDrive(speed, gyroCurve, false);
  	}*/

  }

  
}
