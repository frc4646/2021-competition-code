/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class IndexOne extends CommandBase {
  /**
   * Creates a new IndexOne.
   */
  int currentBalls;
  enum PosStates{A, B, C, D, E, F, G, H, I};
  PosStates currentState;

  public IndexOne() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_conveyor);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (Robot.m_conveyor.getPositions(false, false, false, false))
      {
        currentState = PosStates.A;
      }
      else if (Robot.m_conveyor.getPositions(true, false, false, false))
      {
        currentState = PosStates.B;
      }
      else if (Robot.m_conveyor.getPositions(false, false, true, false))
      {
        currentState = PosStates.C;
      }
      else if (Robot.m_conveyor.getPositions(true, false, true, false))
      {
        currentState = PosStates.D;
      }
      else if (Robot.m_conveyor.getPositions(false, false, true, true))
      {
        currentState = PosStates.G;
      }
      else if (Robot.m_conveyor.getPositions(true, false, true, true))
      {
        currentState = PosStates.H;
      }
      else if (Robot.m_conveyor.getPositions(true, true, true, true))
      {
        currentState = PosStates.I;
      }
      else {
        System.out.println("Something went wrong with assigning our states.");
        currentState = PosStates.A;
      }

      switch (currentState)
      {
        case A:
          while (!Robot.m_conveyor.isLowBallPresent()) 
          {
            Robot.m_conveyor.UpBottomConveyor();
          }
          Robot.m_conveyor.StopBottomConveyor();
          end(true);
          break;
        case B:
          while (!Robot.m_conveyor.isHighBallPresent())
          {
            Robot.m_conveyor.UpBottomConveyor();
            Robot.m_conveyor.UpTopConveyor();
          }
          Robot.m_conveyor.StopBottomConveyor();
          Robot.m_conveyor.StopTopConveyor();
          end(true);
          break;
        case C:
          while (!Robot.m_conveyor.isLowBallPresent())
          {
            Robot.m_conveyor.UpBottomConveyor();
          }
          Robot.m_conveyor.StopBottomConveyor();
          end(true);
          break;
        case D:
          while (!Robot.m_conveyor.isMiddleBallPresent())
          {
            Robot.m_conveyor.UpBottomConveyor();
          }
          while (Robot.m_conveyor.isMiddleBallPresent())
          {
            Robot.m_conveyor.UpTopConveyor();
          }
          while (Robot.m_conveyor.isHighBallPresent() && !Robot.m_conveyor.isMiddleBallPresent())
          {
            Robot.m_conveyor.StopBottomConveyor();
            Robot.m_conveyor.StopTopConveyor();
          }
          end(true);
          break;
        case G:
          while (!Robot.m_conveyor.isLowBallPresent())
          {
            Robot.m_conveyor.UpBottomConveyor();
          }
          Robot.m_conveyor.StopBottomConveyor();
          end(true);
          break;
        case H:
          while (!Robot.m_conveyor.isLowBallPresent() && !Robot.m_conveyor.isMiddleBallPresent())
          {
            Robot.m_conveyor.UpBottomConveyor();
          }
          Robot.m_conveyor.StopBottomConveyor();
          end(true);
          break;
        case I:
          System.out.println("All balls in conveyer! Whooo!");
          end(true);
          break;
        default:
          System.out.println("Something went wrong with our index one switch statement.");
          end(true);
          break;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_conveyor.StopBottomConveyor();
    Robot.m_conveyor.StopTopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
