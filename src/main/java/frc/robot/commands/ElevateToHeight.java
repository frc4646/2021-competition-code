/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ElevateToHeight extends CommandBase {
  /**
   * Creates a new ElevatorUp.
   */
  private double wantedHeight;
  private double tolerance;

  public ElevateToHeight(double inchesY) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_climber);
    wantedHeight = inchesY;
    tolerance = .5f;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.printf("Intialize%f\n", wantedHeight);

    if (Robot.m_climber.GetLiftHeight() < wantedHeight)
    {
      Robot.m_climber.ElevateBySpeed(Robot.m_climber.UP_POWER);
    }
    else
    {
      Robot.m_climber.ElevateBySpeed(Robot.m_climber.DOWN_POWER);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_climber.HoldHeight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((wantedHeight + tolerance) >= Robot.m_climber.GetLiftHeight() &&
    (wantedHeight - tolerance) <= Robot.m_climber.GetLiftHeight());
  }
}
