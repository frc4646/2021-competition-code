/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
//import frc.robot.commands.ElevatorTeleOp;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  
   VictorSPX winchVictor1;
   VictorSPX winchVictor2;
   Spark elevatorSpark;
   
   public Encoder winchEncoder1;
   public Encoder winchEncoder2;
   AnalogInput liftStringPotPin;

   public final double MAX_VALUE = 0;
   public final double MIN_VALUE = 0;
   public final double MAX_HEIGHT = 0;
   public final double MIN_HEIGHT = 0;
   public final double HOLD_POWER = 0;
   public final double UP_POWER = -0.8;
   public final double DOWN_POWER = 0.5;
   public final double WINCH_POWER = 0.7;

  public Climber() {
    winchVictor1 = new VictorSPX(Constants.winch1Spark);
    winchVictor2 = new VictorSPX(Constants.winch2Spark);

    elevatorSpark = new Spark(Constants.elevatorSpark);
    
    liftStringPotPin = new AnalogInput(Constants.liftStringPotPin);

    //winchEncoder1 = new Encoder(Constants.winch1EncoderPort1, Constants.winch1EncoderPort2);
    //winchEncoder2 = new Encoder(Constants.winch2EncoderPort1, Constants.winch2EncoderPort2);

    //setDefaultCommand(new ElevatorTeleOp());
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ElevatorUp() {
    elevatorSpark.set(UP_POWER);
  }

  public void ElevatorDown() {
    elevatorSpark.set(DOWN_POWER);
  }

  public void ElevatorTeleOp() {
    elevatorSpark.set(0.5);
  }

  public void resetEncoders() {
    //winchEncoder1.reset();
    //winchEncoder2.reset();
  }

  public void WinchTeleOp() {
    winchVictor1.set(ControlMode.PercentOutput, WINCH_POWER);
    winchVictor2.set(ControlMode.PercentOutput, -WINCH_POWER);
  }

  //Goes in both directions
  public void ElevateBySpeed(double speed) {
    elevatorSpark.set(speed);
  }
  public void HoldHeight() {
    elevatorSpark.set(HOLD_POWER);
  }

  public void WinchPull() {
    winchVictor1.set(ControlMode.PercentOutput,WINCH_POWER);
    winchVictor2.set(ControlMode.PercentOutput,WINCH_POWER);
  }

  public void winchStop() {
    winchVictor1.set(ControlMode.PercentOutput,0);
    winchVictor2.set(ControlMode.PercentOutput,0);
  }

  public double GetLiftHeight() {
    double pinVoltage = liftStringPotPin.getVoltage();
    double m = (MIN_HEIGHT - MAX_HEIGHT) / (double)(MIN_VALUE - MAX_VALUE);
    double b = MIN_HEIGHT - ((MIN_VALUE)*(m));
    double height = ((m)*(pinVoltage)) + b;
    return height;
  }

}
