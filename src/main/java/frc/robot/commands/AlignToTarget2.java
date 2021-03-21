// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AlignToTarget2 extends CommandBase {
  /** Creates a new AlignToTarget2. */
  private double rotationSpeed;
  private PIDController controller;

  public AlignToTarget2() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_limelight);
    addRequirements(Robot.m_drivetrain);

    controller = new PIDController(0.1f, 0f, 0f);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setTolerance(0.1f);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.m_limelight.IsTrackingTarget()) {
      rotationSpeed = controller.calculate(Robot.m_limelight.TargetPos()[0], 0);
      System.out.println("Rotation speed: " + rotationSpeed);
    }
    else {
      rotationSpeed = 0f;
      System.out.println("Cannot find target.");
      end(false);
    }
    Robot.m_drivetrain.driveByPercent(rotationSpeed, -rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("At setpoint.");
    Robot.m_drivetrain.driveByPercent(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
