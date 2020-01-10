/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class MoveToLevel extends InstantCommand {
  /**
   * Add your docs here.
   */
  int level;

  public MoveToLevel(int level) {
    super();
    this.level = level;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (level == 1) {// Cargo 1
      RobotMap.targetEncoderValue = RobotMap.CargoLevel1TargetValue;
    } else if (level == 2) {// Cargo 2
      RobotMap.targetEncoderValue = RobotMap.CargoLevel2TargetValue;
    } else if (level == 3) {// Cargo 3
      RobotMap.targetEncoderValue = RobotMap.CargoLevel3TargetValue;
    } else if (level == 4) {// Hatch 1
      RobotMap.targetEncoderValue = RobotMap.HatchLevel1TargetValue;
    } else if (level == 5) {// Hatch 2
      RobotMap.targetEncoderValue = RobotMap.HatchLevel2TargetValue;
    } else if (level == 6) {// Hatch 3
      RobotMap.targetEncoderValue = RobotMap.HatchLevel3TargetValue;
    } else if (level == 7) {// Cargo ship drop in
      RobotMap.targetEncoderValue = RobotMap.CargoShipDropPoint;
    } else if (level == 8) {// Straight Up
      RobotMap.targetEncoderValue = RobotMap.StraightUp;
    } else if (level == 9) {// Cargo from the floor
      RobotMap.targetEncoderValue = RobotMap.CargoFloor;
    } else if (level == 10) {// Cargo from loading station
      RobotMap.targetEncoderValue = RobotMap.CargoLoadingStation;
    } else if (level == 12) {// Hatch from loading station
      RobotMap.targetEncoderValue = RobotMap.HatchLevel1TargetValue;
    } else if (level == 13) {// climbing position
      RobotMap.targetEncoderValue = RobotMap.backToClimb;
    }
  }

}
