
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import  frc.robot.RobotMap;
import frc.robot.commands.FlyWheelDrive;

public class GameMech extends Subsystem {
	private final  DoubleSolenoid panelShooter = new DoubleSolenoid(RobotMap.gmForwardChannel, RobotMap.gmReverseChannel);
	
	public void initDefaultCommand() {
		setDefaultCommand(new FlyWheelDrive());
		open();
	}

	public void open() {
		panelShooter.set(DoubleSolenoid.Value.kReverse);
	}
	
	public void pull(){
		panelShooter.set(DoubleSolenoid.Value.kForward);
	}
	// shift the gearbox to the opposite state
	public void toggleMechPiston(){
		if (panelShooter.get()==DoubleSolenoid.Value.kForward){
			pull();
		}else {
			open();
		}
	}

	public void spinFlyWheels(double speed){
		RobotMap.flywheel1.set(ControlMode.PercentOutput, -speed);
		RobotMap.flywheel2.set(ControlMode.PercentOutput, -speed);

		if (speed < 0 && getCargoLimitSwitch()) {
			pull();
		}
	}

	public double getFlywheel1Encoder(){
		return RobotMap.flywheel1.getSelectedSensorPosition();
	}

	public double getFlywheel2Encoder(){
		return RobotMap.flywheel2.getSelectedSensorPosition();
	}
	
	public boolean getCargoLimitSwitch(){
		boolean cargoIn = RobotMap.cargoIntakeLimitSwitch.get();
		//SmartDashboard.putBoolean("Cargo Limit Switch", cargoIn);
		return cargoIn;
	}
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	// make sure the pistons are closed at first
}
