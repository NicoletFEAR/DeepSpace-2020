/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class PlaceCargo extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PlaceCargo(int level) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    double distOffset = RobotMap.getDistanceOffset(level);
    addSequential(new MoveToLevel(8));
    //addSequential(new DriveArc(2, 4, 20, 12));
    //addSequential(new DriveArc(Robot.x_val_target, Robot.y_val_target, Robot.angle_val_target, distOffset));
    addSequential(new DriveToTarget(Robot.x_val_target, Robot.y_val_target, Robot.angle_val_target, distOffset));
    addSequential(new MoveToLevel(level));
    addParallel(new FlyWheelSetSpeed(.5));
  }
}
