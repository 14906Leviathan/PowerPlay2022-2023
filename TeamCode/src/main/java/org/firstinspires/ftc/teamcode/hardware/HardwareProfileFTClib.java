package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;


public class HardwareProfileFTClib {


    /*
     * Constants
     */
    public final double PIVOT_SPEED = 0.5;
    public final double COUNTS_PER_ROTATION = 28;
    public final double GB_COUNTS_PER_ROTATION = 28;    // goBilda encoder value
    public final double MIN_PIDROTATE_POWER = 0.15;

    /*
     *  Constants & variables for wheel parameters
     */
    public final double DRIVE_TICKS_PER_INCH = 47;
    public final double STRAFE_FACTOR = 0.9;

    public final int LIFT_RESET = 0;
    public final int LIFT_MIN_LOW = 0;
    public final int LIFT_MAX_HIGH = 4200;
    public final int LIFT_LOW_JUNCTION = 1800;
    public final int LIFT_MID_JUNCTION = 3000;
    public final int LIFT_HIGH_JUNCTION = 4100;
    public final int LIFT_EXTRACT_CONE = 1100;
    public final int LIFT_CONE5 = 680;
    public final int LIFT_CONE4 = 550;
    public final int LIFT_CONE3 = 300;
    public final int LIFT_CONE2 = 200;
    public final double LIFT_POWER = 1;
    public final double SERVO_GRAB_OPEN = 0.3;
    public final double SERVO_GRAB_CLOSE = 0.65;

    public final double PID_Kp = 0.08;
    public final double PID_Ki = 0.01;
    public final double PID_Kd = 0.000001;
    public final double PID_MIN_SPEED = 0.05;
    public final double PID_ROTATE_ERROR = 1;

    public final double DRIVE_Kp = 0.05;
    public final double DRIVE_Ki = 0.01;
    public final double DRIVE_Kd = 0.31;

    /*
     * Hardware devices
     */
    public DcMotorEx motorBase = null;
    public DcMotor lampRobot = null;

    public DistanceSensor sensorWall = null;

    public RevIMU imu = null;
    public Servo servoGrabber;
    public RevBlinkinLedDriver LEDPort;
    public DistanceSensor sensorJunction;
    public DistanceSensor sensorJunction2;

    public MotorEx motorLF = null;
    public MotorEx motorLR = null;
    public MotorEx motorRF = null;
    public MotorEx motorRR = null;

    public MecanumDrive mecanum = null;

    HardwareMap hwMap;

    /*
     * Declare Odometry hardware
     */

    /* Constructor */
    public HardwareProfileFTClib() {
    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;


        motorLF = new MotorEx(ahwMap, "motorLF", Motor.GoBILDA.RPM_312);
        motorLF.setRunMode(Motor.RunMode.VelocityControl);
//        motorLF.setVeloCoefficients(DRIVE_Kp, DRIVE_Ki, DRIVE_Kd);
        motorLF.setInverted(true);
        motorLF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorLR = new MotorEx(ahwMap, "motorLR", Motor.GoBILDA.RPM_312);
        motorLR.setRunMode(Motor.RunMode.VelocityControl);
//        motorLR.setVeloCoefficients(DRIVE_Kp, DRIVE_Ki, DRIVE_Kd);
        motorLR.setInverted(true);
        motorLR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorRF = new MotorEx(ahwMap, "motorRF", Motor.GoBILDA.RPM_312);
        motorRF.setRunMode(Motor.RunMode.VelocityControl);
//        motorRF.setVeloCoefficients(DRIVE_Kp, DRIVE_Ki, DRIVE_Kd);
        motorRF.setInverted(false);
        motorRF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorRR = new MotorEx(ahwMap, "motorRR", Motor.GoBILDA.RPM_312);
        motorRR.setRunMode(Motor.RunMode.VelocityControl);
//        motorRR.setVeloCoefficients(DRIVE_Kp, DRIVE_Ki, DRIVE_Kd);
        motorRR.setInverted(false);
        motorRR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        //drivebase init
        mecanum = new MecanumDrive(motorLF, motorRF, motorLR, motorRR);

        /*
        List<LynxModule> allHubs = ahwMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

         */


        motorBase = ahwMap.get(DcMotorEx.class,"motorBase");
        motorBase.setDirection(DcMotor.Direction.REVERSE);
        motorBase.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBase.setTargetPosition(0);
        motorBase.setPower(0);
        motorBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lampRobot = ahwMap.dcMotor.get("RobotLamp");
        lampRobot.setPower(0);


        /***
         * initialize sensors
         */
//        sensorWall = hwMap.get(DistanceSensor.class, "Wall");


        /*
         * Initialize LED Controller
        LEDPort = hwMap.get(RevBlinkinLedDriver.class, "LEDPort");
        LEDPort.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
         */


        servoGrabber = ahwMap.servo.get("servoGrabber");

        sensorJunction = ahwMap.get(DistanceSensor.class, "sensorJunction");
        sensorJunction2 = ahwMap.get(DistanceSensor.class, "sensorJunction2");


        /*
         * Initialize Sensors
         **/

        // imu init

        imu = new RevIMU(ahwMap);
        imu.init();



        /* Webcam device will go here */
//        webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }


    /**
     * The RunMode of the motor.
     */
    public enum RunMode {
        VelocityControl, PositionControl, RawPower
    }
}
