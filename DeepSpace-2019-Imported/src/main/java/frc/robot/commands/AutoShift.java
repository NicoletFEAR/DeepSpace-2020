/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.SensorCollection;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class AutoShift extends Command {
  public AutoShift() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.shifter);

  }

  SensorCollection sensorLeft;
  SensorCollection sensorRight;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    sensorLeft = RobotMap.left1.getSensorCollection();
    sensorRight = RobotMap.right1.getSensorCollection();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // shifting
    double averageVelocity = (Math.abs(sensorLeft.getQuadratureVelocity())
        + Math.abs(sensorRight.getQuadratureVelocity())) / 2;

    SmartDashboard.putNumber("averageVelocity", averageVelocity);

    if (!(Robot.oi.xbox1.getStartButton())) { // check the driver isn't holding down the low gear button
      if (averageVelocity < RobotMap.SHIFT_DOWN_THRESHOLD) { // if not in low, switch to low
        if (Robot.shifter.shifty.get() != DoubleSolenoid.Value.kForward) {
          Robot.shifter.shiftdown();
        }
      } else if (averageVelocity > RobotMap.SHIFT_UP_THRESHOLD) { // if in low, switch to high
        if (Robot.shifter.shifty.get() == DoubleSolenoid.Value.kForward) {
          Robot.shifter.shiftup();
        }
      }
    } else {
      if (Robot.shifter.shifty.get() != DoubleSolenoid.Value.kForward) {
        Robot.shifter.shiftdown();
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
