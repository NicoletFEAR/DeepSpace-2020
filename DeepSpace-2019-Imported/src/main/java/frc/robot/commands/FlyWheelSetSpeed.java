/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class FlyWheelSetSpeed extends InstantCommand {
    /**
     * Add your docs here.
     */
    private double speed;
    public FlyWheelSetSpeed(double speed) {
        super();
        requires(Robot.gameMech);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        this.speed = speed;
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
        Robot.gameMech.spinFlyWheels(speed);
        try {
            Thread.sleep(500);
        } catch (Exception e) {
            System.out.println("Could make flywheel thread sleep!");
        }
        Robot.gameMech.spinFlyWheels(0);
    }
}
