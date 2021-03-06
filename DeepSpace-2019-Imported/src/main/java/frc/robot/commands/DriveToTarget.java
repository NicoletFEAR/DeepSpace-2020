// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Command handling driving in an arc
 */
public class DriveToTarget extends Command {
    double x;
    double y;
    double theta;

    double z;
    double r;

    double currentLength;
    double prefferedLength;

    double error;
    double speed;
    double derivative;
    double integral;
    double previousError;

    boolean complete;
    boolean left;

    public DriveToTarget(double x, double y, double theta, double distOffset) {
        requires(Robot.driveTrain);
        this.x = x - distOffset * Math.cos(Math.toRadians(90 - theta));
        this.y = (y * 12) - distOffset * Math.sin(Math.toRadians(90 - theta));
        this.theta = -theta; // inverted

        setInterruptible(true);
    }

    public double TicksToRevolution(double numberOfTicks) {
        // System.out.println("TicksToRevolution()");
        double percentRotation = numberOfTicks / RobotMap.WHEEL_TICKS_PER_REVOLUTION;
        return percentRotation;
    }

    public double RevolutionsToInches(double percentRotation) {
        // System.out.println("RevolutionToInches()");
        double distanceTraveled = (2 * Math.PI * RobotMap.WHEEL_RADIUS) * percentRotation;
        return distanceTraveled;
    }

    @Override
    protected void initialize() {
        complete = false;
        
        //reset navX for tuning arc
        Robot.navX.reset();

        prefferedLength = y / Math.cos(Math.toRadians(theta));
        currentLength = Math.sqrt(x * x + y * y);

        // boolean turnBool = false;
        // while (!turnBool) {
        //     turnBool = Robot.driveTrain.turnToAngle(theta);
        //     SmartDashboard.putString("Turning", "Turn");
        // }

        // End of Turning to Angle Logic

        SmartDashboard.putString("Turning", "done turning");

        // reset the encoders
        Robot.driveTrain.resetEncoders();
    } // end of init, go to execute

    @Override
    protected void execute() {
        SmartDashboard.putNumber("Speed", speed);

        arcDriveRacing(currentLength);
        Robot.driveTrain.RacingDrive(speed, 0);

        SmartDashboard.putBoolean("Complete arc drive", complete);

    }

    double currentLocation;

    // Distance away should be in inches
    public void arcDriveRacing(double distanceAway) {
        //average both encoders
        double encoderAverage = Math.abs(Robot.driveTrain.getLeftEncoderPosition()) +
            Math.abs(Robot.driveTrain.getRightEncoderPosition()) / 2.0;
        currentLocation = RevolutionsToInches(TicksToRevolution(encoderAverage));
        
        currentLocation = Math.abs(currentLocation);

        SmartDashboard.putNumber("Arc target", distanceAway);
        SmartDashboard.putNumber("currentDistance", currentLocation);

        //PID measurements
        error = distanceAway - currentLocation;
        integral += error * .02;
        derivative = (error - previousError) / .02;
        previousError = error;

        SmartDashboard.putNumber("Drive Arc Error", error);

        //  Welcome to the Amazing World of PID :D
        speed = RobotMap.DRIVE_kP * error + RobotMap.DRIVE_kI * integral + RobotMap.DRIVE_kD * derivative;
        if (Math.abs(error) < 10)
            complete = true;
    }

    @Override
    protected boolean isFinished() {
        return complete || Robot.oi.getXbox1().getTriggerAxis(GenericHID.Hand.kLeft) != 0 || Robot.oi.getXbox1().getTriggerAxis(GenericHID.Hand.kRight) != 0;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // sub//Systems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
