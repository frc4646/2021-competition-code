/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Launcher extends SubsystemBase {
  /**
   * Creates a new Lawn Chair.
   */
  CANSparkMax launcherSpark;
  CANEncoder launcherEncoder;

  //int deviceID;
  //CANSparkMaxLowLevel.MotorType type = CANSparkMaxLowLevel.MotorType.kBrushless;
  double launchSpeed;
  double encoderCountsPerInch;
  
  private final PIDController launch_PID;

  public double launch_kP;
  public double launch_kI;
  public double launch_kD;
  public double launch_tolerance;
  public double launch_derivativeTolerance;
  public double launch_error;

  public double launchSpeedSmartDashboard;

  public Launcher() {

    launcherSpark = new CANSparkMax(Constants.launcherID, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    launcherSpark.restoreFactoryDefaults();

    launcherSpark.setInverted(false);

    launcherSpark.burnFlash();
    
    launchSpeed = 0.8; //this is temporary, we'll find the right number through trial and error?
    launcherEncoder = launcherSpark.getEncoder();

    launch_kP = .3; launch_kI = 0; launch_kD = 0;
    launch_tolerance = 1;
    launch_derivativeTolerance = .01;
    //launch_error = -launcherEncoder.getRate();

    launch_PID = new PIDController(launch_kP, launch_kI, launch_kD);
    launch_PID.setTolerance(launch_tolerance, launch_derivativeTolerance);

    SmartDashboard.putNumber("Launch Speed", 0.8f);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    launchSpeedSmartDashboard = SmartDashboard.getNumber("Launch Speed", 0.8f);
  }
  
  public void StopLauncher() {
    launcherSpark.set(0);
  }

  public double getSpeed() {
    return launcherEncoder.getVelocity();
  }
  
  public void setSpeed(double speed){
    launcherSpark.set(speed);
  }
 
  public boolean atSetpoint()
  {
    return launch_PID.atSetpoint();
  }

  public void resetPID()
  {
    launch_PID.reset();
  }
}
