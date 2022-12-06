package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class HardwareProfile {


    /*
     * Constants
     */
    public final double COUNTS_PER_INCH = 307.699557;
    public final double PIVOT_SPEED = 0.5;
    public final double COUNTS_PER_ROTATION = 28;
    public final double GB_COUNTS_PER_ROTATION = 28;    // goBilda encoder value
    public final double MIN_PIDROTATE_POWER = 0.2;

    /*
     *  Constants & variables for wheel parameters
     */
    private final double DRIVE_TICKS_PER_ROTATION = 537.6;       // from goBilda website
    private final double WHEEL_RADIUS = 1.968505;         // 100mm wheel in inches
    private final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;    // circumference in inches
    public final double DRIVE_TICKS_PER_INCH = DRIVE_TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;

    public final double TARGET_SHOOTER_RPM = 5500;       // from goBilda website
    public final double AUTO_SHOOTER_RPM = 5200;
    public final double AUTO_SHOOTER_PS_RPM = 5000;

    public final double SERVO_GRAB_OPEN = 0.3;
    public final double SERVO_GRAB_CLOSE = 0.6;
    public final double SERVO_LINEAR_INITIALIZED = 0.75;
    public final double SERVO_LINEAR_HG_SHOOT = 0.46;
    public final double SERVO_LINEAR_TELEOP_SHOOT = 0.40;
    public final double SERVO_LINEAR_REDHG_SHOOT = 0.46;
    public final double SERVO_LINEAR_PS_SHOOT = 0.35;
    public final double SERVO_LINEAR_INTAKE = 0.65;
    public final double SERVO_LINEAR_AUTO_SHOOT = 0.5;
    public final double SERVO_WOBBLE_GRAB_OPEN = 0.5;
    public final double SERVO_WOBBLE_GRAB_CLOSE = 0.9;


    /*
     * Hardware devices
     */
    public DcMotor motorLF = null;  // Left Front Drive Motor
    public DcMotor motorLR = null;  // Left Rear  Drive Motor
    public DcMotor motorRF = null;  // Right Front Drive Motor
    public DcMotor motorRR = null;  // Right Rear Drive Motor
    public DcMotorEx motorBase = null;

    public DistanceSensor sensorWall = null;

    /*
    public DcMotor motorShooter1 = null;    // High Speed motor for the shooter
    public DcMotor motorShooter2 = null;    // Second high Speed motor for the shooter

     */

    public BNO055IMU imu;       // Internal accelerometer / Gyro sensor

//    public Servo servoGrab;   // Servo that grabs the wobble goal
    public Servo servoGrabber;
//    public Servo servoRwheel;
//    public Servo servoLinear;

/*
    public Servo servoWobbleArm1;   // First servo that controls the Wobble Goal Arm
    public Servo servoWobbleArm2;   // second servo that controls the Wobble Goal arm
    public Servo servoIntake;   // Servo that deploys the intake mechanism

    public TouchSensor touchLiftUp;
    public TouchSensor touchLiftDown;

    public ColorSensor colorStone;

    public DistanceSensor distanceFront;

    public WebcamName webcam;

     */

    public RevBlinkinLedDriver LEDPort;

    /*
     * Declare Odometry hardware
     */
//        public DcMotor verticalOdometryLeft = null;
//        public DcMotor verticalOdometryRight = null;
//        public DcMotor horizontalOdometry = null;


    /* Constructor */
    public HardwareProfile() {

    }

    public void init(HardwareMap ahwMap) {

        HardwareMap hwMap;
        hwMap = ahwMap;

        /*
         * Initialize Motors
         */

        motorLF = hwMap.dcMotor.get("motorLF");
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setPower(0);

        motorLR = hwMap.dcMotor.get("motorLR");
        motorLR.setDirection(DcMotor.Direction.REVERSE);
        motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLR.setPower(0);

        motorRF = hwMap.dcMotor.get("motorRF");
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setPower(0);

        motorRR = hwMap.dcMotor.get("motorRR");
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRR.setPower(0);

     /*   motorTurnTable = hwMap.dcMotor.get("motorTurnTable");
        motorTurnTable.setDirection(DcMotor.Direction.FORWARD);
        motorTurnTable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTurnTable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTurnTable.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTurnTable.setPower(0);

        //motorArm = hwMap.get(DcMotorEx.class, "motorArm");
        //motorArm.setDirection(DcMotor.Direction.REVERSE);
       // motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  motorArm.setTargetPosition(0);
      //  motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //  motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      //  motorArm.setPower(0);
*/
        motorBase = hwMap.get(DcMotorEx.class,"motorBase");
        motorBase.setDirection(DcMotor.Direction.REVERSE);
        motorBase.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorBase.setTargetPosition(0);
        motorBase.setPower(0);
        motorBase.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /***
         * initialize sensors
         */
//        sensorWall = hwMap.get(DistanceSensor.class, "Wall");

/***
        motorIntake = hwMap.dcMotor.get("motorIntake");
        motorIntake.setDirection(DcMotor.Direction.FORWARD);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setPower(0);

        motorShooter = hwMap.get(DcMotorEx.class, "motorShooter");
        motorShooter.setDirection(DcMotor.Direction.FORWARD);
        motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter.setPower(0);


        motorIntake = hwMap.dcMotor.get("motorIntake");
        motorIntake.setDirection(DcMotor.Direction.FORWARD);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setPower(0);

        motorShooter2 = hwMap.dcMotor.get("motorShooter2");
        motorShooter2.setDirection(DcMotor.Direction.REVERSE);
        motorShooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShooter2.setPower(0);

        /*
         * Initialize Odometry Encoders
        verticalOdometryLeft = hwMap.dcMotor.get("odometryLeft");
        verticalOdometryLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalOdometryLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalOdometryRight = hwMap.dcMotor.get("odometryRight");
        verticalOdometryRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalOdometryRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        horizontalOdometry = hwMap.dcMotor.get("odometryHorizontal");
        horizontalOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdometry.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       */

        /*
         * Initialize LED Controller
        LEDPort = hwMap.get(RevBlinkinLedDriver.class, "LEDPort");
        LEDPort.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
         */


        servoGrabber = hwMap.servo.get("servoGrabber");
        /*
//        servoWobbleGrab.setPosition(0.05);
        servoWobbleArm1 = hwMap.servo.get("servoWobbleArm1");
        servoWobbleArm2 = hwMap.servo.get("servoWobbleArm2");
        servoIntake = hwMap.servo.get("servoIntake");

         */

        /*
         * Initialize Sensors
         **/
        imu = hwMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        /* Webcam device will go here */
//        webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }

}
