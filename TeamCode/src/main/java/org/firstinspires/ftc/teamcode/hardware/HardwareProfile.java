package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.libs.DriveMecanum;

import java.util.List;


public class HardwareProfile {


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
    public final double DRIVE_TICKS_PER_INCH = 41.6;
    public final double STRAFE_FACTOR = 0.9;

    public final int LIFT_RESET = 0;
    public final int LIFT_MIN_LOW = 0;
    public final int LIFT_MAX_HIGH = 4200;
    public final int LIFT_LOW_JUNCTION = 1700;
    public final int LIFT_MID_JUNCTION = 2700;
    public final int LIFT_HIGH_JUNCTION = 4000;
    public final int LIFT_CONE5 = 630;
    public final int LIFT_CONE4 = 500;
    public final int LIFT_CONE3 = 300;
    public final int LIFT_CONE2 = 200;
    public final double LIFT_POWER = 1;
    public final double SERVO_GRAB_OPEN = 0.3;
    public final double SERVO_GRAB_CLOSE = 0.65;

    public final double PID_Kp = 0.08;
    public final double PID_Ki = 0.01;
    public final double PID_Kd = 0.000001;
    public final double PID_MIN_SPEED = 0.05;
    public final double PID_ROTATE_ERROR = 0.5;

    /*
     * Hardware devices
     */
    public DcMotorEx motorLF = null;  // Left Front Drive Motor
    public DcMotorEx motorLR = null;  // Left Rear  Drive Motor
    public DcMotorEx motorRF = null;  // Right Front Drive Motor
    public DcMotorEx motorRR = null;  // Right Rear Drive Motor
    public DcMotorEx motorBase = null;
    public DcMotor lampRobot = null;

    public DistanceSensor sensorWall = null;

    public RevIMU imu = null;

    //    public BNO055IMU imu;       // Internal accelerometer / Gyro sensor
    public Servo servoGrabber;
    public Servo servoAlign;
    public RevBlinkinLedDriver LEDPort;
    public DistanceSensor sensorJunction;
    public DistanceSensor sensorJunction2;

    /*
     * Declare Odometry hardware
     */

    /* Constructor */
    public HardwareProfile() {
    }

    public void init(HardwareMap ahwMap) {
        HardwareMap hwMap;
        hwMap = ahwMap;
        List<LynxModule> allHubs = hwMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        /*
         * Initialize Motors
         */

        motorLF = hwMap.get(DcMotorEx.class, "motorLF");
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setPower(0);

        motorLR = hwMap.get(DcMotorEx.class, "motorLR");
        motorLR.setDirection(DcMotor.Direction.REVERSE);
        motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLR.setPower(0);

        motorRF = hwMap.get(DcMotorEx.class, "motorRF");
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setPower(0);

        motorRR = hwMap.get(DcMotorEx.class, "motorRR");
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setPower(0);

        motorBase = hwMap.get(DcMotorEx.class, "motorBase");
        motorBase.setDirection(DcMotor.Direction.REVERSE);
        motorBase.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBase.setTargetPosition(0);
        motorBase.setPower(0);
        motorBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lampRobot = hwMap.dcMotor.get("RobotLamp");
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

        servoGrabber = hwMap.servo.get("servoGrabber");

        sensorJunction = hwMap.get(DistanceSensor.class, "sensorJunction");
        sensorJunction2 = hwMap.get(DistanceSensor.class, "sensorJunction2");
        /*
         * Initialize Sensors
         **/

        // imu init
        imu = new RevIMU(hwMap);
        imu.init();

        /*
        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

         */

        /* Webcam device will go here */
//        webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }
}

