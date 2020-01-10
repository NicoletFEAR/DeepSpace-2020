package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PressureSensor extends Subsystem {
	public AnalogInput ai = new AnalogInput(0);

	// Put methods for controlling this subsystem
	public int getPressure() {
		double pressure = 250.0 * ai.getVoltage() / 5.0 - 25.0;
		boolean enoughPressure = true;
		SmartDashboard.putNumber("Pressure", pressure);
		// return pressure;
		if (pressure < 60) {
			enoughPressure = false;
		}
		SmartDashboard.putBoolean("Pressure is not enough", enoughPressure);
		return (int) pressure;

	}

	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}