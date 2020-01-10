/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class ArcDrive2 extends InstantCommand {

  double x;
  double y;
  double distOffset;

  int level;

  public ArcDrive2(int level) {

    super();

    // Uses input from the Android vision system to drive to targets and place game
    // pieces
    x = Robot.x_val_target;
    y = Robot.y_val_target;
    distOffset = RobotMap.getDistanceOffset(level);

    this.level = level;

    requires(Robot.driveTrain);

  }

  // Called once when the command executes
  @Override
  protected void initialize() {

    setDriving();

    setHeight();

    if (Robot.doneArc) {
      operateMech();
    }

  }

  public void setDriving() {
    SmartDashboard.putNumber("DistOffset", distOffset);

    if (((y < distOffset + RobotMap.adjustmentAllowance) && (Math.abs(x) < RobotMap.adjustmentAllowance)) || Robot.isTargetNull) {
      Robot.doneArc = true;
    } else {
      Robot.doneArc = false;
    }

    SmartDashboard.putBoolean("DoneArc", Robot.doneArc);

    double output = (y - distOffset) * RobotMap.y_multiplier;
    double turn = x * RobotMap.x_multiplier;

    if (turn > RobotMap.xMaxTurnSpeed)
      turn = RobotMap.xMaxTurnSpeed;
    else if (turn < -RobotMap.xMaxTurnSpeed)
      turn = -RobotMap.xMaxTurnSpeed;

    if (Robot.doneArc) {
      Robot.driveTrain.stop();
    } else {
      Robot.driveTrain.RacingDrive(output, turn);
    }
  }

  public void setHeight() {

    if (y < distOffset + 12) {
      new MoveToLevel(level);
    } else {
      new MoveToLevel(8);
    }

  }

  public void operateMech() {
    if (level == 1 || level == 2 || level == 3 || level == 7) { // if we want to shoot a cargo
      new FlyWheelSetSpeed(0.3);
    } else if (level == 4 || level == 5 || level == 6) { // we want to place a hatch
      new GameMechClose();
    } else if (level == 12) { // we want to get from the loading station
      new GameMechOpen();
    }
  }

}
