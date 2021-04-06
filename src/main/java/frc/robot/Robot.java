/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;
import frc.robot.commandGroups.*;
import frc.robot.commands.DriveTeleOp;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.CvSink;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static Drivetrain m_drivetrain;
  public static Conveyor m_conveyor;
  public static Climber m_climber;
  public static Intake m_intake;
  public static Launcher m_launcher;
  public static Limelight m_limelight;
  //public static PhotonVision m_photonVision;
  public static RobotContainer m_robotContainer;

  public static DriveToLaunch m_autonomousDriveToLaunch;

  CvSink cvSink;
  //CvSource outputStream;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    m_drivetrain = new Drivetrain();
    m_conveyor = new Conveyor();
    m_climber = new Climber();
    m_intake = new Intake();
    m_launcher = new Launcher();
    m_limelight = new Limelight();
    //m_photonVision = new PhotonVision();
    m_robotContainer = new RobotContainer();

    m_autonomousDriveToLaunch = new DriveToLaunch();
    
    CameraServer.getInstance().startAutomaticCapture();
    cvSink = CameraServer.getInstance().getVideo();
    //outputStream = CameraServer.getInstance().putVideo("Camera", 640, 480);
    //outputStream.setFPS(20);

    m_autonomousCommand = m_autonomousDriveToLaunch;
    
    Robot.m_drivetrain.resetEncoders();
    Robot.m_drivetrain.zeroHeading();
    Robot.m_drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    Robot.m_drivetrain.resetEncoders();
    Robot.m_drivetrain.zeroHeading();
    Robot.m_drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    Robot.m_drivetrain.resetEncoders();
    Robot.m_drivetrain.zeroHeading();
    Robot.m_drivetrain.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, new DriveTeleOp());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //I think this is how it's done?

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
