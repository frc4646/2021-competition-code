/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
      //Drivetrain
//   public static final int frontLeftDrivePort = 2;  This stuff is the actual competition ports
//   public static final int frontRightDrivePort = 1; 
//   public static final int backLeftDrivePort = 3; 
//   public static final int backRightDrivePort = 0; 
  public static final int frontLeftDrivePort = 0;  //talon; this stuff is for when I (Prithvi) tested the code on Demobot
  public static final int frontRightDrivePort = 1; //talon
  public static final int backLeftDrivePort = 2;   //victor
  public static final int backRightDrivePort = 3;  //victor


      //Intake
  public static final int intakePort = 2;
  public static final int articulateIntakePort = 4;
  public static final int leftEncoderPort1 = 0;
  public static final int leftEncoderPort2 = 1;
  public static final int rightEncoderPort1 = 2;
  public static final int rightEncoderPort2 = 3;
  

      //Climber
  public static final int winch1Spark = 4; //Know
  public static final int winch2Spark = 5; //Know
  public static final int elevatorSpark = 0;
  public static final int liftStringPotPin = 3;
  public static final int winch1EncoderPort1 = 4;
  public static final int winch1EncoderPort2 = 5;
  public static final int winch2EncoderPort1 = 6;
  public static final int winch2EncoderPort2 = 7;
  
      //Launcher

  public static final int launcherID = 8; //Known
  public static final int PAN_PORT = 5;
  public static final int TILT_PORT = 6;

      //Conveyor
  public static final int frontConveyorPort = 1;
  public static final int rearConveyorPort = 3;
  public static final int lowOpticPort = 7;
  public static final int middleOpticPort = 8;
  public static final int highOpticPort = 9;
  public static final int launcherOpticPort = 10;


      //Joysticks
  public static final int leftJoyPort = 0;
  public static final int rightJoyPort = 1;
  public static final int mechJoyPort = 2;
}