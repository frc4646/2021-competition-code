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
    /*if (Robot.m_vision.IsTrackingTarget()) {
      System.out.println("Found target.");
      if (Robot.m_vision.TargetPos()[0] < -3) {
          System.out.println("Turn right.");
          Robot.m_drivetrain.driveByPercent(.5, -.5);
      }      
      else if (Robot.m_vision.TargetPos()[0] > 3) {
          System.out.println("Turn left.");
          Robot.m_drivetrain.driveByPercent(-.5, .5);         
      }
    }
    else {
      System.out.println("Cannot find target.");
    }*/
    Robot.m_drivetrain.driveByAngle(Robot.m_limelight.TargetPos()[0]);
    System.out.println("Running AlignToTarget");
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
    /*if (Robot.m_vision.TargetPos()[0] > -3
    || Robot.m_vision.TargetPos()[0] < 3)
    {
      System.out.println("AlignToTarget task completed.");
      return true;
    }
    else
    {
      return false;
    }*/
    
    if (Robot.m_drivetrain.atTargetAngle())
    {
      System.out.println("At Target Angle: Is Finished Returns True");
      System.out.println("AlignToTarget Finished");
      return true;
    }
    return false;
  }
}
