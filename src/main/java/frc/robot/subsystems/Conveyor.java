/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  /**
   * Creates a new Conveyor.
   */
  Spark topConveyor;
  Spark bottomConveyor;
  double topConveyorUpSpeed;
  double bottomConveyorUpSpeed;
  double topConveyorDownSpeed;
  double bottomConveyorDownSpeed;

  DigitalInput lowOptic;
  DigitalInput middleOptic;
  DigitalInput highOptic;
  DigitalInput launcherOptic;

  public Conveyor() {
    topConveyor = new Spark (Constants.frontConveyorPort);
    bottomConveyor = new Spark (Constants.rearConveyorPort);
    topConveyor.setInverted(true);
    bottomConveyor.setInverted(true);
    topConveyorUpSpeed = 0.7;
    topConveyorDownSpeed = -0.5;
    bottomConveyorUpSpeed = 0.8; //Was .5
    bottomConveyorDownSpeed = -0.3;
    lowOptic = new DigitalInput(Constants.lowOpticPort);
    middleOptic = new DigitalInput(Constants.middleOpticPort);
    highOptic = new DigitalInput(Constants.highOpticPort);
    launcherOptic = new DigitalInput(Constants.launcherOpticPort);
  }

  public void UpTopConveyor() {
    topConveyor.set(topConveyorUpSpeed);
  }

  public void DownTopConveyor() {
    topConveyor.set(topConveyorDownSpeed);
  }

  public void UpBottomConveyor() {
    bottomConveyor.set(bottomConveyorUpSpeed);
  }

  public void DownBottomConveyor() {
    bottomConveyor.set(bottomConveyorDownSpeed);
  }

  public void StopTopConveyor() {
    topConveyor.set(0);
  }

  public void StopBottomConveyor() {
    bottomConveyor.set(0);
  }

  public boolean isLowBallPresent() {
    return lowOptic.get();
  }

  public boolean isMiddleBallPresent()
  {
    return middleOptic.get();
  }

  public boolean isHighBallPresent() {
    return highOptic.get();
  }

  public boolean isLauncherBallPresent() {
    return launcherOptic.get();
  }
  
  boolean[] getFourPosition()
  {
    boolean[] pos = {isLowBallPresent(), isMiddleBallPresent(), isHighBallPresent(), isLauncherBallPresent()};
    return pos;
  }

  public boolean getPositions(boolean index1, boolean index2, boolean index3, boolean index4)
  {
    boolean[] wantedOnes = {index1, index2, index3, index4};
    return wantedOnes == getFourPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
