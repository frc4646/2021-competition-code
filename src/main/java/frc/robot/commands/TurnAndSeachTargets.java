// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TurnAndSeachTargets extends CommandBase {
  /** Creates a new TurnAndSeachTargets. */
  /*double wantedAngle;
  double largestTargetArea;
  public TurnAndSeachTargets() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_drivetrain);
    addRequirements(Robot.m_photonVision);
    wantedAngle = 180;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.m_drivetrain.driveToHeading(wantedAngle);

    if (Robot.m_photonVision.HasTargets()) {
      double currentTargetSize = Robot.m_photonVision.TargetSize();
      if (largestTargetArea < currentTargetSize)
        largestTargetArea = currentTargetSize;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_photonVision.setLargestTargetArea(largestTargetArea);
    Robot.m_drivetrain.driveByPercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Robot.m_drivetrain.atTargetAngle();
  }*/
}