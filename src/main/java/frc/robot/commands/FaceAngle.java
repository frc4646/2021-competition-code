/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class FaceAngle extends CommandBase {
  /**
   * Creates a new FaceAngle.
   */

  private double wantedAngle;
  private boolean isWantedAngle;
  private double tolerance;
  private double leftPower;
  private double rightPower;

  public FaceAngle(double degree) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_drivetrain);
    wantedAngle = degree;
    tolerance = 2.5;
    leftPower = 0.5f;
    rightPower = 0.5f;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.m_drivetrain.resetGyro();
    isWantedAngle = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroAngle = Robot.m_drivetrain.getAngle();

    if (gyroAngle - tolerance < wantedAngle) //If gyro angle is less than the wanted angle:
    {
      Robot.m_drivetrain.driveByPercent(leftPower, -rightPower); //Turn right
    }
    else if (gyroAngle + tolerance > wantedAngle) //If gyro angle is greater than the wanted angle:
    {
      Robot.m_drivetrain.driveByPercent(-leftPower, rightPower); //Turn left
    }
    else
    {
      isWantedAngle = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_drivetrain.driveByPercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isWantedAngle;
  }
}
