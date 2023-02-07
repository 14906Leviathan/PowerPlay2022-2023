package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.hardware.HardwareProfileFTClib;
import org.firstinspires.ftc.teamcode.libs.DriveMecanumFTCLib;

import java.util.List;

@Autonomous(name = "Auto - Squid Red Terminal Feed Forward", group = "Leviathan")
//@TeleOp(name = "Auto - Squid Red Terminal Feed Forward", group = "Leviathan")

public class LevRedTerminalFeedForward extends LinearOpMode {

    private final static HardwareProfileFTClib robot = new HardwareProfileFTClib();
    private LinearOpMode opMode = this;

    FtcDashboard dashboard;
    public static double l1_Kp = 0.05;
    public static double l2_Ki = 0.001;
    public static double l3_Kd = 0.01;
    public static double l4_MIN_SPEED = 0.10;
//    public static double l20_Kp = 0.02;
//    public static double l21_Ki = 0.001;
//    public static double l22_Kd = 0.001;
//    public static double l23_MIN_SPEED = 0.18;

    private static final String TFOD_MODEL_ASSET = "GenericSignalSleeve.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "circle",
            "star",
            "triangle"
    };

    private static final String VUFORIA_KEY =
        "ARLYRsf/////AAABmWpsWSsfQU1zkK0B5+iOOr0tULkAWVuhNuM3EbMfgb1+zbcOEG8fRRe3G+iLqL1/iAlTYqqoLetWeulG8hkCOOtkMyHwjS/Ir8/2vUVgC36M/wb9a7Ni2zuSrlEanb9jPVsNqq+71/uzTpS3TNvJI8WeICQNPAq3qMwmfqnCphVlC6h2ZSLsAR3wcdzknFmtpApdOp1jHJvITPeD/CMdAXjZDN0XJwJNQJ6qtaYSLGC23vJdQ2b1aeqnJauOvswapsG7BlmR7m891VN92rNEcOX7WmMT4L0JOM0yKKhPfF/aSROwIdNtSOpQW4qEKVjw3aMU1QDZ0jj5SnRV8RPO0hGiHtXy6QJcZsSj/Y6q5nyf";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    private ElapsedTime runtime = new ElapsedTime();

    private boolean blueAlliance = false;       //red if false, blue if true

    /* Declare OpMode members. */
//    public HardwareProfileFTClib robot = new HardwareProfileFTClib();
//    public LinearOpMode opMode = this;
    private State setupState = State.ALLIANCE_SELECT;     // default setupState configuration
    private State runState = State.SET_DISTANCES;
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    int position = 2;

    public LevRedTerminalFeedForward(){

    }

//    @Override
    public void runOpMode() {

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.

         */

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.motorBase.setTargetPosition(0);
//        robot.lampRobot.setPower(1);
        //robot.motorArm.setTargetPosition(0);
        robot.motorBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        while(!isStarted() && !isStopRequested()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                        double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                        double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                        double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                        telemetry.addData(""," ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                        telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);

                        if(recognition.getLabel() == "circle"){
                            position =1;
                        } else if(recognition.getLabel() == "triangle" ){
                            position = 2;
                        } else position = 3;
                    }
                    telemetry.addData("sensor Junction", robot.sensorJunction.getDistance(DistanceUnit.INCH));
                    telemetry.addData("sensor Junction2", robot.sensorJunction2.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                    // post telemetry to FTC Dashboard as well
                    /*
                    dashTelemetry.put("p01 - PID IMU Angle X                  = ", robot.imu.getAngles()[0]);
                    dashTelemetry.put("p02 - PID IMU Angle Y                  = ", robot.imu.getAngles()[1]);
                    dashTelemetry.put("p03 - PID IMU Angle Z                  = ", robot.imu.getAngles()[2]);
                    dashTelemetry.put("p04 - Lift Front Encoder Value = ", robot.motorBase.getCurrentPosition());
                    dashTelemetry.put("p06 - sensor Junction", robot.sensorJunction.getDistance(DistanceUnit.INCH));
                    dashTelemetry.put("p07 - sensor Junction2", robot.sensorJunction2.getDistance(DistanceUnit.INCH));
                    dashboard.sendTelemetryPacket(dashTelemetry);

                     */
                }
            }

        }  // end of while

        if(isStopRequested()) requestOpModeStop();   // user requested to abort setup
        robot.lampRobot.setPower(0);

        runtime.reset();
        runState = State.LEVEL_ADJUST;
        double turnAngle = 90;

        while (opModeIsActive()) {
            switch(runState){
                case TEST:

//                    drive.ftclibDrive(0, 50);
                    drive.ftclibRotate(45, 1);
                    sleep(2000);

                    drive.ftclibRotate(0, 1);
                    sleep(2000);

                    drive.ftclibRotate(-45, 1);
                    sleep(2000);

                    drive.ftclibRotate(0, 1);
                    sleep(2000);

                    drive.ftclibRotate(90, 1);
                    sleep(2000);

                    drive.ftclibRotate(0, 1);
                    sleep(2000);

                    drive.ftclibRotate(-90, 1);
                    sleep(2000);

                    drive.ftclibRotate(0, 1);
                    sleep(2000);

                    drive.ftclibRotate(135, 1);
                    sleep(2000);

                    drive.ftclibRotate(0, 1);
                    sleep(2000);

                    drive.ftclibRotate(-135, 1);
                    sleep(2000);

                    drive.ftclibRotate(135, 1);
                    sleep(2000);

                    drive.ftclibRotate(0, 1);
                    sleep(2000);

                    runState = State.HALT;
                    break;

                case LEVEL_ADJUST:
                    robot.lampRobot.setPower(0);
                    runState = State.HIGH_JUNCTION_1;
                    break;

                case HIGH_JUNCTION_1:
                    // starting from start position, close claw
                    drive.closeClaw();

                    // Drive forward away from wall, pushing signal cone out of position

                    drive.ftclibDrive(0, 61);

                    // raise the arm to position the cone
                    drive.liftHighJunction();

                    //back up to position to score cone
                    drive.ftclibDrive(180,5);

                    //turn to high junction
                    drive.ftclibRotate(45,robot.PID_ROTATE_ERROR);

                   //Wait for the lift to raise
                    sleep(300);

                    // Drive forward to the high junction
                    drive.ftclibDrive(0,4);

                    // drive forward to detect the junction
                    drive.detectJunction(0.2, 0.75);

                    // lower the arm and release the cone
                    drive.liftMidJunction();
                    sleep(300);

                    drive.openClaw();

                    // raise the lift to keep from entangling on junction
                    drive.liftHighJunction();
                    sleep(300);
                    drive.liftReset();

                    // back away from the junction
                    drive.ftclibDrive(180, 6);

                    // reset the lift to its starting position
                    drive.liftReset();

                    //rotate towards the cone stack
                    drive.ftclibRotate(-90, robot.PID_ROTATE_ERROR);

                    runState = State.CONE_2;
                    break;

                case CONE_2:
                    //rotate towards the cone stack
                    drive.ftclibRotate(-90, robot.PID_ROTATE_ERROR);

                    // lower the arm to pick up the top cone
                    drive.liftPosition(robot.LIFT_CONE5);

                    //drive towards the stack of cones
                    drive.ftclibDrive(0,16);

                    // adjust direction - turn towards cone stack
                    drive.ftclibRotate(-90, robot.PID_ROTATE_ERROR);

                    //drive towards the stack of cones
                    drive.ftclibDrive(0,13);

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(700);

                    //back away from the wall slightly
                    drive.ftclibDrive(180,0.5);

                    // lift the cone up to clear the stack
                    drive.liftLowJunction();
                    sleep(600);

                    runState = State.LOW_JUNCTION_2;
                    break;

                case LOW_JUNCTION_2:
                    // back away to tile 2
                    drive.ftclibDrive(180,23);

                    // rotate towards the low junction
                    drive.ftclibRotate(-135, robot.PID_ROTATE_ERROR);

                    // drive towards the junction
                    drive.ftclibDrive(0, 4);

                    // drive forward to detect the junction
                    drive.detectJunction(0.2, 2);

                    // place the cone
                    drive.liftPosition(robot.LIFT_RESET);
                    sleep(400);
                    drive.openClaw();

                    // raise the lift to clear the junction
                    drive.liftLowJunction();
                    sleep(300);

                    // back away from the junction
                    drive.ftclibDrive(180, 9);

                    // turn towards the stack
                    drive.ftclibRotate(-90, robot.PID_ROTATE_ERROR);

                    runState = State.CONE_3;
                    break;

                case CONE_3:
                    // lower the arm to pick up the top cone
                    drive.liftPosition(robot.LIFT_CONE4);

                    //drive towards the stack of cones
                    drive.ftclibDrive(0,15);

                    // adjust direction - turn towards cone stack
                    drive.ftclibRotate(-90, robot.PID_ROTATE_ERROR);

                    //drive towards the stack of cones
                    drive.ftclibDrive(0,12);

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(600);

                    //back away from the wall slightly
                    drive.ftclibDrive(180,0.5);

                    // lift the cone up to clear the stack
                    drive.liftLowJunction();
                    sleep(600);

                    // back away to tile 2
                    drive.ftclibDrive(180,26);

                    runState = State.MID_JUNCTION_3;
                    break;

                case LOW_JUNCTION_3:
                    // rotate towards the low junction
                    drive.ftclibRotate(-125, robot.PID_ROTATE_ERROR);

                    // drive towards the junction
                    drive.ftclibDrive(0, 3);

                    // place the cone
                    drive.liftReset();
                    sleep(400);
                    drive.openClaw();

                    // raise the lift to clear the junction
                    drive.liftLowJunction();

                    // turn towards the stack
                    drive.ftclibRotate(-90, robot.PID_ROTATE_ERROR);

                    runState = State.PARK;
                    break;

                case MID_JUNCTION_3:
                    // rotate towards the low junction
                    drive.ftclibRotate(-225, robot.PID_ROTATE_ERROR);

                    // raise the arm to position the cone
                    drive.liftMidJunction();

                    //Wait for raise
                    sleep(500);

                    // Drive forward to the high junction
                    drive.ftclibDrive(0,3);

                    //drive forward to detect the junction
                    drive.detectJunction(0.2, 2);

                    // lower the arm and release the cone
                    drive.liftLowJunction();
                    sleep(400);

                    drive.openClaw();

                    // raise the lift to keep from entangling on junction
                    drive.liftMidJunction();

                    // back away from the junction
                    drive.ftclibDrive(180, 5);

                    //rotate towards the cone stack
                    drive.ftclibRotate(90, robot.PID_ROTATE_ERROR);

                    // reset the lift to its starting position
                    drive.liftReset();

                    runState = State.PARK;
                    break;

                case PARK:

                    if(position == 3) {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the stack to park - ready to grab the first cone in teleop
                        //drive.PIDRotate(-90, robot.PID_ROTATE_ERROR);

                        // drive to park position 1
                        drive.ftclibDrive(0,24);

                    } else if (position == 2) {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the outside wall position
                        //drive.PIDRotate(-90, robot.PID_ROTATE_ERROR);

                        // drive to park position 1
                        drive.ftclibDrive(180,2);

                    } else {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the outside wall position
                        //drive.PIDRotate(-90, robot.PID_ROTATE_ERROR);

                        // drive to park position 1
                        drive.ftclibDrive(180,18);
                    }

                    while(opModeIsActive() && robot.motorBase.getCurrentPosition() > 10){
                        drive.liftReset();
                    }

                    runState = State.HALT;

                    break;


                case HALT:

                    // shut down all motors
                    drive.motorsHalt();

                    requestOpModeStop();    // request stoppage of the program

                    break;
            }   // end of switch(state)
        }   // end of while(opModeIsActive)

        requestOpModeStop();

        telemetry.addData("Path", "Complete");
        telemetry.update();

    } // end of opmode

    /*
     * Enumerate the states of the machine
     */
    enum State {
        TEST, ALLIANCE_SELECT, HIGH_JUNCTION_1, CONE_2, LOW_JUNCTION_2, CONE_3, LOW_JUNCTION_3, MID_JUNCTION_3, LEVEL_ADJUST, PARK, HALT, SET_DISTANCES
    }   // end of enum State

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

       parameters.vuforiaLicenseKey = VUFORIA_KEY;
       parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
       vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

}       //End Linear Op Mode
