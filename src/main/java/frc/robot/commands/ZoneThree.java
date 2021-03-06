/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

public class ZoneThree extends CommandBase {
  /**
   * Creates a new SpinLauncher.
   */
  double launcherSpeed = .8; //placeholder .8

  public ZoneThree() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_launcher);
    addRequirements(Robot.m_conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_launcher.setSpeed(0.725f);
    Timer.delay(1.5f);
    Robot.m_conveyor.UpTopConveyor();
    Robot.m_conveyor.UpBottomConveyor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Robot.m_conveyor.UpConveyor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_conveyor.StopTopConveyor();
    Robot.m_conveyor.StopBottomConveyor();
    Robot.m_launcher.StopLauncher();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}