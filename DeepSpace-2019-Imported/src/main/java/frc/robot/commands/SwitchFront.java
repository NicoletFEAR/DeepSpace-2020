package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;

public class SwitchFront extends InstantCommand {

	public SwitchFront() {
        super();
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	//Flip direction of travel
        Robot.driveTrain.switchFront();

        // if (Robot.cameraMode.equalsIgnoreCase("front")) {
        //     Robot.mVisionServer.backCamera();
        //     Robot.cameraMode = "back";
        //   } else {
        //     Robot.mVisionServer.frontCamera();
        //     Robot.cameraMode = "front";
        //   }

        if (Robot.driveTrain.isReversed()) {
            Robot.mVisionServer.frontCamera();
            Robot.cameraMode = "front";
        } else {
            Robot.mVisionServer.backCamera();
            Robot.cameraMode = "back";
        }

    	//Flip left and right
        //Robot.oi.switchJoystickIDs();
    }
}
