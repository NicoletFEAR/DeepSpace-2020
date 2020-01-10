package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// DRIVE BASE:
	public static TalonSRX left1;
	public static TalonSRX left2;
	public static TalonSRX left3;

	public static TalonSRX right1;
	public static TalonSRX right2;
	public static TalonSRX right3;

	public static final double DRIVE_LIMITER = 0.75; // not 775 motors

	public static final double SHIFT_UP_THRESHOLD = 1750; // speed at which drive base shifts up
	public static final double SHIFT_DOWN_THRESHOLD = 1900; // speed at which drive base shifts down

	public static final double DRIVE_RAMP_RATE = 10; // 10 is recomended by our friends from 2015, need to test

	// CARGO HATCH GAME MECH:
	public static TalonSRX flywheel1;
	public static TalonSRX flywheel2;

	public static DigitalInput cargoIntakeLimitSwitch; // intake limit switch
	public static int CARGO_LIMIT_SWITCH_INPUT = 0;

	public static final double FLYWHEEL_LIMITER = 0.75; // 775 motors

	// ARM:
	public static TalonSRX armMotor1;
	public static TalonSRX armMotor2;

	public static final double ARM_LIMITER = 0.4; // not 775 motors

	public static double ARM_MOTOR_SLOW_FORWARDS = 0.928; // slow the faster motor to match the slower 
	public static double ARM_MOTOR_SLOW_BACKWARDS = 0.967; // slow the faster motor to match the slower

	// PID CONSTANTS:
	public static double ERROR_CONSTANT_LEFT = 0.0;
	public static double ERROR_CONSTANT_RIGHT = 0.0;
	public static double WHEEL_RADIUS = 2.0; // Inches // UDB:2 // FCB:4
	public static double WHEEL_SEPARATION = 5.0; // Inches
	public static double DISTANCE_BETWEEN_TRACKS = 20.0; // Inches // FCB:20.5 // UDB:20
	public static double fudgeFactor = 1.0;
	public static double WHEEL_TICKS_PER_REVOLUTION = 7610;
	public static final double DRIVE_kP = 0.03;
	public static final double DRIVE_kI = 0;
	public static final double DRIVE_kD = 0;
	public static final double DRIVE_ERROR_CONSTANT = 0;
	public static final double TURN_kP = 0.04;
	public static final double TURN_kI = 0.001;
	public static final double TURN_kD = 0.001;
	public static final double TURN_ERROR_CONSTANT = 2; // degrees the turn can be off
	public static final double PERFECT_ARC_RANGE = 2;
    public static final double TURN_SCALING = 0.6;

	// ARM MOVEMENT CONSTANTS:
	public static int maxChangeAmt = 100; // how much the arm moves by
	public static int targetEncoderValue;
	public static final int CargoShipDropPoint = -1350;
	public static final int CargoLevel1TargetValue = -1750; //-2000
	public static final int CargoLevel2TargetValue = -1100; //-1200;
	public static final int CargoLevel3TargetValue = -560; //-900;
	public static final int HatchLevel1TargetValue = -2125; //-2200;
	public static final int HatchLevel2TargetValue = -1225; //-1400;
	public static final int HatchLevel3TargetValue = -660; //-1100;
	public static final int CargoLoadingStation = -2100; //-1200;
	public static final int CargoFloor = -2400; //-3500;
	public static final int StraightUp = 0;
	public static final int backToClimb = 1100;
	// P: increases proportional to error
	public static final double ARM_kP = 0.002; // please raise
	// I: sum of error over time, helps arm get to final pos 
	public static final double ARM_kI = 0.0001; // 
	// D: slows arm down when it's too fast
	public static final double ARM_kD = 0.00005; // please raise
	public static double ARM_TICKS_PER_REVOLUTION = 4096.0; // Needs to be updated on final bot!!!!!!!!!!!!!!!!!!!!!!!!
	public static int ARM_MAX_TICK_VAL = 2750;
	public static int ARM_MIN_TICK_VAL = -2750;
	public static int offset = 0;
	public static double ARM_DEAD_ZONE = 1; // check

	public static double ARM_JOYSTICK_DEAD_ZONE = 0.05; // check

	// VISION CONSTANTS:
	public static double angleConstant = 2400;
	public static double distanceConstant = 1.5;
	public static double x_val_constant = .95;
	public static double visionXAllowance = 14;

	// ARC DRIVE 2.0 CONSTANTS
	public static double y_multiplier = 0.03; // multiplies the target output by the Android vision to robot driving output
	public static double x_multiplier = 0.03;
	public static double adjustmentAllowance = 1.5;
	public static double xMaxTurnSpeed = 0.25;

	// // ultrasonic sensors
	// public static AnalogInput ultraLeft;
	// public static AnalogInput ultraRight;

	public static int gmModuleNumber = 0; // GAME MECH
	public static int gmForwardChannel = 3;
	public static int gmReverseChannel = 4;

	public static int shiftModuleNumber = 0; // SHIFTER
	public static int shiftForwardChannel = 0;
	public static int shiftReverseChannel = 7;

	public static int compressormodule = 0;

	//distance offsets for arc + angle drive
	public static double cargo1Dist = 45;
	public static double cargo2Dist = 38;
	public static double cargo3Dist = 45;
	public static double hatch1Dist = 38;
	public static double hatch2Dist = 34;
	public static double loadingStationHatchDist = 38;
	public static double loadingStationCargoDist = 38;
	public static double cargoShipCargoDist = 35;
	public static double cargoShipHatchDist = 38;
	

	public static void init() {
		targetEncoderValue = 0;
		left1 = new TalonSRX(15);
		left1.setNeutralMode(NeutralMode.Brake);
		left2 = new TalonSRX(13);
		left2.setNeutralMode(NeutralMode.Brake);
		left3 = new TalonSRX(17);
		left3.setNeutralMode(NeutralMode.Brake);

		// Set how fast of a rate the robot will accelerate Do not remove or you
		// get a fabulous prize of a Flipping robot - 2015
		left1.configClosedloopRamp(DRIVE_RAMP_RATE);
		left2.configClosedloopRamp(DRIVE_RAMP_RATE);
		left3.configClosedloopRamp(DRIVE_RAMP_RATE);

		left1.setInverted(true);
		left2.setInverted(true);
		left3.setInverted(true);
		left2.follow(left1);
		left3.follow(left1);

		right1 = new TalonSRX(16);
		right1.setNeutralMode(NeutralMode.Brake);
		right2 = new TalonSRX(14);
		right2.setNeutralMode(NeutralMode.Brake);
		right3 = new TalonSRX(18);
		right3.setNeutralMode(NeutralMode.Brake);

		// Set how fast of a rate the robot will accelerate Do not remove or you
		// get a fabulous prize of a Flipping robot - 2015
		right1.configClosedloopRamp(DRIVE_RAMP_RATE);
		right2.configClosedloopRamp(DRIVE_RAMP_RATE);
		right3.configClosedloopRamp(DRIVE_RAMP_RATE);

		right1.setInverted(true);
		right2.setInverted(true);
		right3.setInverted(true);
		right2.follow(right1);
		right3.follow(right1);

		flywheel1 = new TalonSRX(25); // cargo handlers
		flywheel2 = new TalonSRX(26);

		flywheel1.setInverted(false); // two flywheels must be opposite to intake and shoot cargo
		flywheel2.setInverted(true);
		flywheel1.setNeutralMode(NeutralMode.Coast);
		flywheel2.setNeutralMode(NeutralMode.Coast);

		cargoIntakeLimitSwitch = new DigitalInput(CARGO_LIMIT_SWITCH_INPUT);

		// next 2 need to be switched back to 21/22,
		// using them for PID testing
		armMotor1 = new TalonSRX(24);
		armMotor1.setNeutralMode(NeutralMode.Brake);
		armMotor1.setInverted(false);
		armMotor1.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10); // new frame ever X miliseconds
		armMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
		armMotor1.setSelectedSensorPosition(0);
		armMotor2 = new TalonSRX(22); // SHOULD BE 22
		armMotor2.setNeutralMode(NeutralMode.Brake);
		armMotor2.setInverted(true);
		// armMotor2.setInverted(InvertType.FollowMaster);
		// armMotor2.setInverted(false);

		// ultraRight = new AnalogInput(3);
		// ultraLeft = new AnalogInput(2);
	}

	public static double getDistanceOffset(int level) {
		switch (level) {
			case 1:
				return cargo1Dist;
			case 2:
				return cargo2Dist;
			case 3:
				return cargo3Dist;
			case 4:
				return hatch1Dist;
			case 5:
				return hatch2Dist;
			case 7:
				return cargoShipCargoDist;
			case 8:
				return loadingStationCargoDist;
			case 12:
				return loadingStationHatchDist;
			default:
				return 40;

		}
	}
}
