/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  Spark intakeSpark;
  Spark articulateSpark;

  double intakeSpeed;
  double outtakeSpeed;
  double deploySpeed;
  double retractSpeed;
  double deploySeconds;
  double retractSeconds;

  AnalogTrigger opticTrigger;
  AnalogInput opticInput;

  final double enableTrigger = 0, disableTrigger = 0;


  public Intake() {
    intakeSpark = new Spark(Constants.intakePort);
    articulateSpark = new Spark(Constants.articulateIntakePort);
    intakeSpark.setInverted(true);
    opticTrigger = new AnalogTrigger(0);
    opticInput = new AnalogInput(1);
    opticTrigger = new AnalogTrigger(opticInput);

    intakeSpeed = 0.5;
    outtakeSpeed = -0.5;
    deploySpeed = 0.35;
    retractSpeed = -0.50;
    deploySeconds = 2.0;
    retractSeconds = 2.0; 

    opticTrigger.setLimitsVoltage(disableTrigger, enableTrigger);

  }

  @Override
  public void periodic() {
    
  }

  public void IntakeBall() {
    intakeSpark.set(intakeSpeed);
  }

  public void OuttakeBall() {
    intakeSpark.set(outtakeSpeed);
  }
  
  public void StopIntake() {
    intakeSpark.set(0);
  }

  public void stopArticulate() {
    articulateSpark.set(0);
  }

  public void DeployIntake() {
    //just run it for 2 seconds
    articulateSpark.set(deploySpeed);
  }

  public void RetractIntake() {
    //just run it for 2 seconds
    articulateSpark.set(retractSpeed);
  }

  public boolean isBallInIntake(){
    return opticTrigger.getTriggerState();
  }

}
