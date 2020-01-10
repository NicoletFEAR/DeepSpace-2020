/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TurnToAngle extends Command {

  private double angle;
  private boolean toFinish;

  public TurnToAngle(double angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.navX.reset();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    toFinish = Robot.driveTrain.turnToAngle(angle);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (toFinish) {
      Robot.driveTrain.integral = 0;
      Robot.driveTrain.previousDesiredAngle = 0;
      Robot.driveTrain.previousError = 0;
      return true;
    }
    return false || Robot.oi.getXbox1().getTriggerAxis(GenericHID.Hand.kLeft) != 0 || Robot.oi.getXbox1().getTriggerAxis(GenericHID.Hand.kRight) != 0;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
