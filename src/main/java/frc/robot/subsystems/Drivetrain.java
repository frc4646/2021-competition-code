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
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class Drivetrain extends SubsystemBase {

  private final WPI_TalonSRX frontLeftDrive;
  private final WPI_TalonSRX frontRightDrive;
  private final WPI_VictorSPX backLeftDrive;
  private final WPI_VictorSPX backRightDrive;
  private DifferentialDriveOdometry odometry;

  //private final Encoder rightEncoder;
  //private final Encoder leftEncoder;

  private final int encoderCountsPerInch;

  private final AHRS navX;
  private final PIDController turn_PID;

  public double turn_kP;
  public double turn_kI;
  public double turn_kD;
  public double turn_tolerance;
  public double turn_derivativeTolerance;
  public double turn_error;
  int maxEncoderTicks = 2048;
  double circumference = Math.PI * 6 * 0.0254; //pi * distance * inches to meters

  

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

    
    navX = new AHRS();
    navX.reset();

    turn_kP = 1f/30f; turn_kI = 0; turn_kD = 0;
    turn_tolerance = 1;
    turn_derivativeTolerance = .01;
    turn_error = -navX.getRate();
    turn_PID = new PIDController(turn_kP, turn_kI, turn_kD);
    turn_PID.setTolerance(turn_tolerance, turn_derivativeTolerance);

    odometry = new DifferentialDriveOdometry(navX.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Encoder", getDriveEncoderCount()[0]);
    SmartDashboard.putNumber("Right Encoder", getDriveEncoderCount()[1]);

    odometry.update(navX.getRotation2d(), frontLeftDrive.getSelectedSensorPosition()*(circumference/maxEncoderTicks), 
                                          frontRightDrive.getSelectedSensorPosition()*(circumference/maxEncoderTicks));
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

  public void driveToHeading(double targetAngle)
  {
    double calculatedPID = turn_PID.calculate(getAngle(), targetAngle);
    frontLeftDrive.set(ControlMode.PercentOutput, calculatedPID);
    frontRightDrive.set(ControlMode.PercentOutput, -calculatedPID);
  }

  public void driveByVolts(double leftVolts, double rightVolts) {
    frontLeftDrive.setVoltage(leftVolts);
    frontRightDrive.setVoltage(-rightVolts);
  }

  public double[] getDrivePower()
  {
    double[] drivePower = new double[]{frontLeftDrive.get(), frontRightDrive.get()};
    return drivePower;
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
    navX.calibrate();
  }

  public double getAngle(){
    return navX.getAngle();
  }

  public double getHeading() {
    return navX.getRotation2d().getDegrees();
  }

  public void zeroHeading() {
    navX.reset();
  }

  public boolean atTargetAngle()
  {
    return turn_PID.atSetpoint();
  }

  public void resetTurnPID()
  {
    turn_PID.reset();
  }

  public double getTurnRate() {
    return -navX.getRate();
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    odometry.resetPosition(pose, navX.getRotation2d());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftDrive.getSelectedSensorVelocity()*(circumference/maxEncoderTicks)/10, frontRightDrive.getSelectedSensorVelocity()*(circumference/maxEncoderTicks)/10);
  }

  /*public void arcadeDrive(double xSpeed, double zRotation)
  {
    m_drive.arcadeDrive(xSpeed, zRotation);
  }*/

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
