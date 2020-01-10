package frc.robot;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.loops.VisionProcessor;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CompressAir;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GameMech;
import frc.robot.subsystems.PressureSensor;
import frc.robot.subsystems.Shifter;
import frc.robot.vision.VisionServer;
import se.vidstige.jadb.JadbConnection;
import se.vidstige.jadb.JadbDevice;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {

    Command autonomousCommand;
    Command disabledCommand;

    public static DriverStation.Alliance alliance;
    public static String allianceColorVal = "";
    public static String teamSwitchSide = "";
    public static double y_val_target = 0.0;
    public static double z_val_target = 0.0;
    public static double x_val_target = 0.0;
    public static double angle_val_target = 0.0;
    public static boolean isTargetNull = true;
    public static boolean doneArc = false;

    public static OI oi;
    public static DriveTrain driveTrain;
    public static GameMech gameMech;
    public static Arm arm;
    public static PressureSensor pressureSensor;
    public static Shifter shifter;
    public static UsbCamera front;
    public static UsbCamera back;
    public static AHRS navX;
    public static CompressAir compressorOAir;
    // public static ArduinoInterface arduinoLEDInterface;
    // public static ArduinoInterface arduinoCameraInterface;

    public boolean compressorRunning = true;

    private static JadbConnection m_jadb = null;
    private static List<JadbDevice> m_devices = null;
    private static JadbDevice m_currentDevice = null;
    private static int m_nextLocalHostPort = 3800;

    public static String cameraMode = "back";

    public static VisionServer mVisionServer;
    public static boolean xPressed = false;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        RobotMap.init();
        mVisionServer = VisionServer.getInstance();
        mVisionServer.frontCamera();

        // String[] args = {};
        // mVisionServer.main(args);

        mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());

        driveTrain = new DriveTrain();

        gameMech = new GameMech();
        gameMech.pull();

        arm = new Arm();

        pressureSensor = new PressureSensor();

        shifter = new Shifter();
        shifter.shiftdown();

        navX = new AHRS(Port.kMXP);
        
        compressorOAir = new CompressAir();

        front = CameraServer.getInstance().startAutomaticCapture("FRONT", 1);
        back = CameraServer.getInstance().startAutomaticCapture("BACK", 0);

       // front.setConnectionStrategy(edu.wpi.cscore.VideoSource.ConnectionStrategy.kKeepOpen);
       // back.setConnectionStrategy(edu.wpi.cscore.VideoSource.ConnectionStrategy.kKeepOpen);

        // arduinoLEDInterface = new ArduinoInterface(7);
        // arduinoCameraInterface = new ArduinoInterface(6);

        // OI must be constructed after subsystems. If the OI creates Commands
        // (which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.

        oi = new OI();
    }

    /**
     * This function is called when the disabled button is hit. You can use it to
     * reset subsystems before shutting down.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        VisionProcessor processor = (VisionProcessor) mVisionServer.receivers.get(0);
        processor.onLoop(System.currentTimeMillis());
        SmartDashboard.putNumber("NavX Angle: ", navX.getAngle());

        // VisionUpdate update = new VisionUpdate();
        // for (int i = 0; i < update.getTargets().size(); i++) {
        // TargetInfo target = update.getTargets().get(i);
        // System.out.println("Target: " + target.getY() + ", " + target.getZ());
        // }

        // System.out.print(update.getTargets());
        // update.targets; // PROBLEM HERE !!!!!!!!!!!!!!
        // boolean b = (targets_list != null);
        // System.out.println(b);
        /*
         * if (update.targets != null) { //TargetInfo target_Info = targets_list.get(0);
         * TargetInfo target_Info = update.targets.get(0); // Double Y_val =
         * targets_list.get(0).getY(); System.out.println(Y_val); }
         */

    }

    @Override
    public void autonomousInit() {
        mVisionServer.backCamera();
        if (disabledCommand != null)
            disabledCommand.cancel();
        if (autonomousCommand != null)
            autonomousCommand.start();
        teleopInit();
        RobotMap.armMotor1.setSelectedSensorPosition(0);
        RobotMap.armMotor2.setSelectedSensorPosition(0);
        
		RobotMap.flywheel1.setSelectedSensorPosition(0,0,10);
		RobotMap.flywheel2.setSelectedSensorPosition(0,0,10);

        RobotMap.targetEncoderValue = 0;
        RobotMap.offset = 0;
        RobotMap.ARM_MAX_TICK_VAL = 2750;
        RobotMap.ARM_MIN_TICK_VAL = -2750;


        shifter.shiftdown();

        Robot.driveTrain.resetEncoders();
        double velocityRight = Robot.driveTrain.getRightEncoderVelocity();
        double velocityLeft = Robot.driveTrain.getLeftEncoderVelocity();
        SmartDashboard.putNumber("velR", velocityRight);
        SmartDashboard.putNumber("velL", velocityLeft);
        SmartDashboard.putNumber("Left Encoder: ", Robot.driveTrain.getLeftEncoderPosition());
        SmartDashboard.putNumber("Right Encoder: ", Robot.driveTrain.getRightEncoderPosition());
    }

    @Override

    public void autonomousPeriodic() {
        teleopPeriodic();
        // double distanceLeft = RobotMap.ultraLeft.getAverageVoltage() * 300 / 293 * 1000 / 25.4;
        // SmartDashboard.putNumber("Distance from left ultrasonic (inches)", distanceLeft);
        // double distanceRight = RobotMap.ultraRight.getAverageVoltage() * 300 / 293 * 1000 / 25.4;
        // SmartDashboard.putNumber("Distance from right ultrasonic (inches)", distanceRight);
    }

    @Override
    public void teleopInit() {
        
    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        pressureSensor.getPressure();
        Scheduler.getInstance().run();
        /*
         * VisionUpdate update = new VisionUpdate();
         * //System.out.print(update.getTargets()); List<TargetInfo> targets_list =
         * update.getTargets(); // PROBLEM HERE !!!!!!!!!!!!!! if (targets_list != null)
         * { TargetInfo target_Info = targets_list.get(0); Double Y_val =
         * targets_list.get(0).getY(); System.out.println(Y_val); }
         */

        VisionProcessor processor = (VisionProcessor) mVisionServer.receivers.get(0);

        processor.onLoop(System.currentTimeMillis());
        // double distanceLeft = RobotMap.ultraLeft.getAverageVoltage() * 300 / 293 * 1000 / 25.4;
        // SmartDashboard.putNumber("Distance from left ultrasonic (inches)", distanceLeft);
        // double distanceRight = RobotMap.ultraRight.getAverageVoltage() * 300 / 293 * 1000 / 25.4;
        // SmartDashboard.putNumber("Distance from right ultrasonic (inches)", distanceRight);

    }

    @Override
    public void robotPeriodic() { // Always runs, good for printing
        SmartDashboard.putNumber("Arm1 Encoder Value", arm.getArm1Encoder());
        SmartDashboard.putNumber("Arm2 Encoder Value", arm.getArm2Encoder());

        SmartDashboard.putNumber("y_val_target: ", y_val_target);
        SmartDashboard.putNumber("z_val_target: ", z_val_target);
        SmartDashboard.putNumber("x_val_target: ", x_val_target);
        SmartDashboard.putNumber("angle_val_target: ", angle_val_target);
        SmartDashboard.putBoolean("Target found: ", !isTargetNull);
        SmartDashboard.putString("Camera Mode: ", cameraMode);

        SmartDashboard.putNumber("Right Encoder: ", Robot.driveTrain.getRightEncoderPosition());
        SmartDashboard.putBoolean("Switch Front", Robot.driveTrain.isReversed());

        double velocityRight = Robot.driveTrain.getRightEncoderVelocity();
        double velocityLeft = Robot.driveTrain.getLeftEncoderVelocity();
        SmartDashboard.putNumber("velR", velocityRight);
        SmartDashboard.putNumber("velL", velocityLeft);

        SmartDashboard.putNumber("Target", RobotMap.targetEncoderValue+RobotMap.offset);

        SmartDashboard.putNumber("NavX", navX.getAngle());
        SmartDashboard.putNumber("Left Encoder: ", Robot.driveTrain.getLeftEncoderPosition());

        SmartDashboard.putNumber("ArmySpeedyBoi", arm.getSpeed());
        
        SmartDashboard.putNumber("Vol_armMotor1", RobotMap.armMotor1.getMotorOutputVoltage());
        SmartDashboard.putNumber("Vol_armMotor2", RobotMap.armMotor2.getMotorOutputVoltage());
        SmartDashboard.putNumber("Vol_left1", RobotMap.left1.getMotorOutputVoltage());
        SmartDashboard.putNumber("Vol_right1", RobotMap.right1.getMotorOutputVoltage());
        SmartDashboard.putNumber("Vol_flywheel1", RobotMap.flywheel1.getMotorOutputVoltage());
        SmartDashboard.putBoolean("armIsManual",Robot.arm.armIsManual);

        SmartDashboard.putNumber("flywheel1", gameMech.getFlywheel1Encoder());
        SmartDashboard.putNumber("flywheel2", gameMech.getFlywheel2Encoder());

        SmartDashboard.putBoolean("Compressor Enabled:", compressorOAir.isEnabled());

        xPressed = oi.getXbox1().getXButton();
        SmartDashboard.putBoolean("Drive X Button: ", xPressed);

        SmartDashboard.putData("Command Running", Scheduler.getInstance());
    }
}
