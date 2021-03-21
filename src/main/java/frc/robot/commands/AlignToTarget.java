// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
/* Procedure
1) Given: Currently tracking a target
2) Find where the target's position on the x-axis is
3) Drive/rotate to the center
*/

public class AlignToTarget extends CommandBase {

  private double headingError;
  private double steeringAdjust;
  private double minCommand = .15f;

  /** Creates a new AlignToTarget. */
  public AlignToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(Robot.m_limelight);
    addRequirements(Robot.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    headingError = Robot.m_limelight.TargetPos()[0];
    steeringAdjust = 0;
    if (Robot.m_limelight.IsTrackingTarget()) {
      System.out.println("Found target.");
      if (headingError >= 1) {
          System.out.println("Turn left.");
          steeringAdjust = Robot.m_drivetrain.turn_kP*headingError+minCommand;
      }      
      else if (headingError <= 1) {
          System.out.println("Turn right.");
          steeringAdjust = Robot.m_drivetrain.turn_kP*headingError-minCommand;
      }
      Robot.m_drivetrain.driveByPercent(-steeringAdjust, steeringAdjust);
      System.out.println("Left Power: " + -steeringAdjust);
      System.out.println("Right Power: " + steeringAdjust);
      System.out.println("Error: " + headingError);
    }
    else {
      System.out.println("Cannot find target.");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_drivetrain.driveByPercent(0, 0);
    System.out.println("AlignToTarget End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Robot.m_limelight.TargetPos()[0] >= -1
    && Robot.m_limelight.TargetPos()[0] <= 1)
    {
      System.out.println("AlignToTarget task completed.");
      Robot.m_drivetrain.driveByPercent(0, 0);
      return true;
    }
    else
    {
      return false;
    }
  }
}
