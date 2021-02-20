/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ReverseBottomConveyer extends CommandBase {
  /**
   * Creates a new ReverseConveyer.
   */
  public ReverseBottomConveyer() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_conveyor.DownBottomConveyor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_conveyor.StopBottomConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
