package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.HardwareProfileFTClib;
import org.firstinspires.ftc.teamcode.libs.DriveMecanumFTCLib;

import java.util.List;

@Autonomous(name = "Auto - Squid Blue Terminal Feed Forward", group = "Leviathan")

public class LevBlueTerminalFeedForward extends LinearOpMode {

    private final static HardwareProfileFTClib robot = new HardwareProfileFTClib();
    private LinearOpMode opMode = this;


    final private boolean distanceSensorFlag = false;


    FtcDashboard dashboard;
    public static double l1_Kp = 0.05;
    public static double l2_Ki = 0.001;
    public static double l3_Kd = 0.01;
    public static double l4_MIN_SPEED = 0.10;

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
    private State setupState = State.ALLIANCE_SELECT;     // default setupState configuration
    private State runState = State.SET_DISTANCES;
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    int position = 3;

    public LevBlueTerminalFeedForward(){

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
        robot.motorBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //

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
                        dashTelemetry.put("p01 - Image", recognition.getLabel());
                        dashTelemetry.put("p02 - confidence level", recognition.getConfidence() * 100 );
                        dashboard.sendTelemetryPacket(dashTelemetry);

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

                    dashTelemetry.put("p03 - PID IMU Angle X                  = ", robot.imu.getAngles()[0]);
                    dashTelemetry.put("p04 - PID IMU Angle Y                  = ", robot.imu.getAngles()[1]);
                    dashTelemetry.put("p05 - PID IMU Angle Z                  = ", robot.imu.getAngles()[2]);
                    dashTelemetry.put("p06 - Lift Front Encoder Value = ", robot.motorBase.getCurrentPosition());
                    dashTelemetry.put("p07 - sensor Junction", robot.sensorJunction.getDistance(DistanceUnit.INCH));
                    dashTelemetry.put("p08 - sensor Junction2", robot.sensorJunction2.getDistance(DistanceUnit.INCH));
                    dashboard.sendTelemetryPacket(dashTelemetry);
                }
            }
        }  // end of while

        if(isStopRequested()) requestOpModeStop();   // user requested to abort setup
        robot.lampRobot.setPower(0);

        runtime.reset();
        runState = State.HIGH_JUNCTION_1;

        while (opModeIsActive()) {
            switch(runState){
                case TEST:
                    runState = State.HALT;
                    break;

                case LEVEL_ADJUST:
                    robot.lampRobot.setPower(0);
                    runState = State.HIGH_JUNCTION_1;
                    break;

                case HIGH_JUNCTION_1:
                    // starting from start position, close claw
                    drive.closeClaw();
                    sleep(150);

                    // raise the arm to position the cone
                    drive.liftHighJunction();
                    drive.alignUp();

                    // Drive forward away from wall, pushing signal cone out of position
                    drive.ftclibDrive(0, 52, 0);

                    //turn to high junction
                    drive.ftclibRotate(-45,robot.PID_ROTATE_ERROR);

                    // decide if the program should use the distance sensors to find the junction
                    if(distanceSensorFlag) {
                        // drive forward to detect the junction
                        drive.detectJunction(0.2, 1.5);

                        // reverse distance slightly
                        drive.ftclibDrive(180, 2, 0);
                    } else {
                        // Drive forward to the high junction
                        drive.ftclibDrive(0,5, 0);
                    }

                    // lower the arm and release the cone
                    drive.alignDown();
                    sleep(400);
                    drive.liftMidJunction();
                    sleep(300);
                    drive.openClaw();

                    // raise the lift to keep from entangling on junction
                    drive.liftHighJunction();

                    // back away from the junction
                    drive.ftclibDrive(180, 7,0);

                    // reset the lift to its starting position
                    drive.liftPosition(robot.LIFT_CONE5);

                    runState = State.CONE_2;
                    break;

                case CONE_2:        // top cone of the starter stack
                    //rotate towards the cone stack
                    drive.ftclibRotate(85, robot.PID_ROTATE_ERROR);

                    // lower the arm to pick up the top cone
                    drive.liftPosition(robot.LIFT_CONE5);

                    //drive towards the stack of cones
                    drive.ftclibDrive(0,30,3);

                    // adjust direction - turn towards cone stack
//                    drive.ftclibRotate(90, robot.PID_ROTATE_ERROR);

                    //drive towards the stack of cones
                    drive.ftclibDrive(180,0.5,0);

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(300);

                    //back away from the wall slightly
                    drive.ftclibDrive(180,1,0);

                    // lift the cone up to clear the stack
                    drive.liftPosition(robot.LIFT_EXTRACT_CONE);
                    sleep(300);

                    runState = State.LOW_JUNCTION_2;
                    break;

                case LOW_JUNCTION_2:    // low junction 1st pass
                    // back away to tile 2
                    drive.ftclibDrive(180,19,3);

                    // lift the rest of the way to low junction
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION);
                    drive.alignUp();

                    // rotate towards the low junction
                    drive.ftclibRotate(135, robot.PID_ROTATE_ERROR);

                    // decide if the program should use the distance sensors to find the junction
                    if(distanceSensorFlag) {
                        // drive forward to detect the junction
                        drive.detectJunction(0.2, 1.5);

                        // reverse distance slightly
                        drive.ftclibDrive(180, 2,0);
                    } else {
                        // Drive forward to the low junction
                        drive.ftclibDrive(0,4,0);
                    }

                    // place the cone
                    drive.alignDown();
                    sleep(400);
                    drive.liftPosition(robot.LIFT_RESET);
                    sleep(200);
                    drive.openClaw();

                    // raise the lift to clear the junction
                    drive.liftLowJunction();
                    sleep(200);

                    // back away from the junction
                    drive.ftclibDrive(180, 4,0);

                    // turn towards the starter stack
                    drive.ftclibRotate(90, robot.PID_ROTATE_ERROR);

                    runState = State.CONE_3;
                    break;

                case CONE_3:        // 4th cone up from starter stack

                    // lower the arm to pick up the top cone
                    drive.liftPosition(robot.LIFT_CONE4);

                    //drive towards the stack of cones
                    drive.ftclibDrive(0,26,3);

                    // adjust direction - turn towards cone stack
//                    drive.ftclibRotate(90, robot.PID_ROTATE_ERROR);

                    /*
                    if(distanceSensorFlag) {
                        // drive forward to detect the junction
                        drive.detectJunction(0.2, 1.5);

                        // reverse distance slightly
                        drive.ftclibDrive(180, 0);
                    } else {
                        // Drive forward to the high junction
                        drive.ftclibDrive(0,11);
                    }

                     */
                    //drive towards the stack of cones
                    drive.ftclibDrive(180,0.5,0);

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(400);

                    //back away from the wall slightly
                    drive.ftclibDrive(180,0.5,0);

                    // lift the cone up to clear the stack
                    drive.liftPosition(robot.LIFT_EXTRACT_CONE);
                    sleep(400);

                    runState = State.LOW_JUNCTION_3;
                    break;

                case LOW_JUNCTION_3:
                    // back away to tile 2
                    drive.ftclibDrive(180,20,0);

                    // lift the rest of the way to low junction
                    drive.liftPosition(robot.LIFT_LOW_JUNCTION);
                    drive.alignUp();

                    // rotate towards the low junction
                    drive.ftclibRotate(135, robot.PID_ROTATE_ERROR);

                    // decide if the program should use the distance sensors to find the junction
                    if(distanceSensorFlag) {
                        // drive forward to detect the junction
                        drive.detectJunction(0.2, 1.5);

                        // reverse distance slightly
                        drive.ftclibDrive(180, 2,0);
                    } else {
                        // Drive forward to the high junction
                        drive.ftclibDrive(0,4,0);
                    }

                    // place the cone
                    drive.alignDown();
                    sleep(400);
                    drive.liftReset();
                    sleep(200);
                    drive.openClaw();

                    // raise the lift to clear the junction
                    drive.liftLowJunction();

                    //back away from the junction
                    drive.ftclibDrive(180, 4,0);

                    // turn towards the stack
                    drive.ftclibRotate(90, robot.PID_ROTATE_ERROR);

                    runState = State.PARK;
                    break;

                case MID_JUNCTION_3:
                    // back away to tile 2
                    drive.ftclibDrive(180,50,0);

                    // raise the arm to position the cone
                    drive.liftMidJunction();

                    // rotate towards the mid junction
                    drive.ftclibRotate(135, robot.PID_ROTATE_ERROR);

                    // decide if the program should use the distance sensors to find the junction
                    if(distanceSensorFlag) {
                        // drive forward to detect the junction
                        drive.detectJunction(0.2, 1.5);

                        // reverse distance slightly
                        drive.ftclibDrive(180, 2,0);
                    } else {
                        // Drive forward to the high junction
                        drive.ftclibDrive(0,8,0);
                    }

                    // lower the arm and release the cone
                    drive.liftLowJunction();
                    sleep(400);
                    drive.openClaw();

                    // raise the lift to keep from entangling on junction
                    drive.liftMidJunction();

                    // back away from the junction
                    drive.ftclibDrive(180, 8,0);

                    //rotate towards the cone stack
                    drive.ftclibRotate(90, robot.PID_ROTATE_ERROR);

                    // reset the lift to its starting position
                    drive.liftReset();

                    runState = State.PARK;
                    break;

                case PARK:

                    drive.alignDown();  // confirm that the claw is down for teleop

                    if(position == 1) {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the stack to park - ready to grab the first cone in teleop
                        //drive.PIDRotate(-90, robot.PID_ROTATE_ERROR);

                        // drive to park position 1
                        drive.ftclibDrive(180,24,0);

                    } else if (position == 2) {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the outside wall position
                        //drive.PIDRotate(-90, robot.PID_ROTATE_ERROR);

                        // drive to park position 2
                        drive.ftclibDrive(180,0,0);

                    } else {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the outside wall position
                        //drive.PIDRotate(-90, robot.PID_ROTATE_ERROR);

                        // drive to park position 3
                        drive.ftclibDrive(0,12,0);

                        drive.ftclibRotate(90, 1);

                        drive.ftclibDrive(0, 12,0);
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

