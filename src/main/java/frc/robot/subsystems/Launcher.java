/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;


public class Launcher extends SubsystemBase {
  /**
   * Creates a new Lawn Chair.
   */
  CANSparkMax launcherSpark1;
  CANSparkMax launcherSpark2;
  CANEncoder launcherEncoder;

  Servo pan, tilt;

  //int deviceID;
  //CANSparkMaxLowLevel.MotorType type = CANSparkMaxLowLevel.MotorType.kBrushless;
  double launchSpeed;
  double encoderCountsPerInch;

  public Launcher() {

    launcherSpark1 = new CANSparkMax(Constants.launcherID1, CANSparkMaxLowLevel.MotorType.kBrushless);
    launcherSpark2 = new CANSparkMax(Constants.launcherID2, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    launcherSpark1.restoreFactoryDefaults();
    launcherSpark2.restoreFactoryDefaults();

    launcherSpark1.setInverted(true);
    launcherSpark2.follow(launcherSpark1, true);

    launcherSpark1.burnFlash();
    launcherSpark2.burnFlash();
    
    launchSpeed = 0.8; //this is temporary, we'll find the right number through trial and error?
    launcherEncoder = launcherSpark1.getEncoder();

    pan = new Servo(Constants.PAN_PORT);
    tilt = new Servo(Constants.TILT_PORT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run() {
    try {
       Thread.sleep(10000);
    } catch (InterruptedException ex) {
       System.out.println("Interrupted");
       Thread.currentThread().interrupt();
       return;
    }
 }
  
  public void StopLauncher() {
    launcherSpark1.set(0);
  }
  public void setServos(double servoPan, double servoTilt) {
    pan.set(servoPan);
    tilt.set(servoTilt);
  }

  public double[] getServoPos() {
    double[] array = {pan.get(), tilt.get()};
    return array;
  }

  public double getSpeed() {
    return launcherEncoder.getVelocity();
  }
  
  public void setSpeed(double speed){
    launcherSpark1.set(speed);
  }

}
