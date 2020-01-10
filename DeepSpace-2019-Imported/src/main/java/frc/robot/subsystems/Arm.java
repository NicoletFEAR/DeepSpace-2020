
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.SensorCollection;

// import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Levels;

/**
 * This subsystem is used for precise positioning of the arm of the robot, and
 * for extending/retracting the arm
 */
public class Arm extends Subsystem {
    double integral = 0;
    double previousError = 0;
    double previousDesiredtargetEncoderValue = 0;
    // int offset=0;
    double speed;
    double derivative;

    double encoderPosition;
    double error;

    public boolean armIsManual = false;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    // make sure the pistons are closed at first
    public void initDefaultCommand() {
        // RobotMap.armMotor1.setSelectedSensorPosition(0,0,10);
        setDefaultCommand(new Levels());
    }

    public void rotateToPosition(double desiredtargetEncoderValue) {
        // checks if the target has changed
        // if it has changed, reset the base variables to 0;
        if (desiredtargetEncoderValue != previousDesiredtargetEncoderValue) {
            integral = 0;
            previousError = 0;
            previousDesiredtargetEncoderValue = desiredtargetEncoderValue;
        }

        SmartDashboard.putNumber("armTarget", desiredtargetEncoderValue);
        encoderPosition = getArm1Encoder();

        error = desiredtargetEncoderValue - encoderPosition;
        integral += error * .02;

        if (Math.abs(error) < 25 && !(Math.abs(error) > Math.abs(previousError))) { // scale down the integral
            integral *= 0.1;
        }

        derivative = (error - previousError) / .02;

        SmartDashboard.putNumber("total_P_arm", RobotMap.ARM_kP * error);
        SmartDashboard.putNumber("total_I_arm", RobotMap.ARM_kI * integral);
        SmartDashboard.putNumber("total_D_arm", RobotMap.ARM_kD * derivative);

        speed = RobotMap.ARM_kP * error + RobotMap.ARM_kI * integral + RobotMap.ARM_kD * derivative;
        speed *= -1;

        if (Math.abs(error) < RobotMap.ARM_DEAD_ZONE) {
            speed = 0;
        }

        if (speed > RobotMap.ARM_LIMITER) {
            speed = RobotMap.ARM_LIMITER;
            // RobotMap.armMotor1.set(ControlMode.PercentOutput, RobotMap.ARM_LIMITER);
            // RobotMap.armMotor2.set(ControlMode.PercentOutput, RobotMap.ARM_LIMITER);
        } else if (speed < -RobotMap.ARM_LIMITER) {
            speed = -RobotMap.ARM_LIMITER;
            // RobotMap.armMotor1.set(ControlMode.PercentOutput, -RobotMap.ARM_LIMITER);
            // RobotMap.armMotor2.set(ControlMode.PercentOutput, -RobotMap.ARM_LIMITER);
        }
        RobotMap.armMotor1.set(ControlMode.PercentOutput, speed);
        // RobotMap.armMotor2.set(ControlMode.PercentOutput, speed);
        if (speed < 0) {
            RobotMap.armMotor2.set(ControlMode.PercentOutput, speed * RobotMap.ARM_MOTOR_SLOW_BACKWARDS);
        } else {
            RobotMap.armMotor2.set(ControlMode.PercentOutput, speed * RobotMap.ARM_MOTOR_SLOW_FORWARDS);
        }

        previousError = error;

    }

    public void rotateNoPID(double desiredtargetEncoderValue) {
        encoderPosition = getArm1Encoder();
        error = desiredtargetEncoderValue - encoderPosition;

        if (error > 200) {
            speed = -1.0;
        } else if (error < -200) {
            speed = 1.0;
        } else if (error < -50) {
            speed = 0.2;
        } else if (error > 50) {
            speed = -0.2;
        } else {
            speed = 0.0;
        }

        SmartDashboard.putNumber("SPD_ARM_NO_PID", 0);
        RobotMap.armMotor1.set(ControlMode.PercentOutput, speed);
        if (speed < 0) {
            RobotMap.armMotor2.set(ControlMode.PercentOutput, speed * RobotMap.ARM_MOTOR_SLOW_BACKWARDS);
        } else {
            RobotMap.armMotor2.set(ControlMode.PercentOutput, speed * RobotMap.ARM_MOTOR_SLOW_FORWARDS);
        }
        // RobotMap.armMotor2.set(ControlMode.PercentOutput, speed *
        // RobotMap.ARM_LIMITER);

    }

    public void manualControl(double joystickInput) {
        RobotMap.armMotor1.set(ControlMode.PercentOutput, joystickInput);
        RobotMap.armMotor2.set(ControlMode.PercentOutput, joystickInput);
        if (joystickInput < 0) {
            RobotMap.armMotor2.set(ControlMode.PercentOutput, joystickInput * RobotMap.ARM_MOTOR_SLOW_BACKWARDS);
        } else {
            RobotMap.armMotor2.set(ControlMode.PercentOutput, joystickInput * RobotMap.ARM_MOTOR_SLOW_FORWARDS);
        }
    }

    public double getArm1Encoder() {
        return RobotMap.armMotor1.getSelectedSensorPosition(); // negative because enoder happens to be the poother way
    }

    public double getArm2Encoder() {
        return RobotMap.armMotor2.getSelectedSensorPosition(); // negative because enoder happens to be the poother way
    }

    public double getSpeed() {
        return speed;
    }

}
