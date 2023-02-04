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
import org.firstinspires.ftc.teamcode.hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.libs.DriveMecanum;

import java.util.List;

@Autonomous(name = "Auto - Squid Blue Terminal Side", group = "Leviathan")

public class LevBlueTerminalSquid extends LinearOpMode{

    FtcDashboard dashboard;

    /*      Original
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
     */

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
    private HardwareProfile robot   = new HardwareProfile();
    private LinearOpMode opMode = this;
    private State setupState = State.ALLIANCE_SELECT;     // default setupState configuration
    private State runState = State.SET_DISTANCES;
    private DriveMecanum drive = new DriveMecanum(robot, opMode);
    int position = 2;
    /* Declare DataLogger variables */

    @Override
    public void runOpMode() {

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        boolean running = true;
        long startDelay = 0;

        // set default values
        int scoreLevel = 1;

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
        robot.init(hardwareMap);
        robot.motorBase.setTargetPosition(robot.LIFT_RESET);
        robot.motorBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lampRobot.setPower(1);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.addData("sensor Junction", String.format("%.01f in", robot.sensorJunction.getDistance(DistanceUnit.INCH)));
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
                        telemetry.addData("sensor Junction", String.format("%.01f in", robot.sensorJunction.getDistance(DistanceUnit.INCH)));
                        telemetry.addData("sensor Junction2", String.format("%.01f in", robot.sensorJunction2.getDistance(DistanceUnit.INCH)));


                        dashTelemetry.put(""," ");
                        dashTelemetry.put("i01 - Pattern Identified         = ", recognition.getLabel());
                        dashTelemetry.put("i02 - Confidence Level           = ", recognition.getConfidence() * 100 );
                        dashTelemetry.put("i03 - Park Position              = ", position);
                        dashTelemetry.put("p00 - PIDTurn Telemetry Data", "");
                        dashTelemetry.put("p01 - PID IMU Angle X                  = ", robot.imu.getAngles()[0]);
                        dashTelemetry.put("p02 - PID IMU Angle Y                  = ", robot.imu.getAngles()[1]);
                        dashTelemetry.put("p03 - PID IMU Angle Z                  = ", robot.imu.getAngles()[2]);
                        dashTelemetry.put("p09 - Right Front                  = ", robot.motorRF.getCurrentPosition());
                        dashTelemetry.put("p10 - Right Rear                   = ", robot.motorRR.getCurrentPosition());
                        dashTelemetry.put("p11 - Left Front                   = ", robot.motorLF.getCurrentPosition());
                        dashTelemetry.put("p12 - Right Rear                   = ", robot.motorRR.getCurrentPosition());
                        dashboard.sendTelemetryPacket(dashTelemetry);

                        if(recognition.getLabel() == "star"){
                            position =3;
                        } else if(recognition.getLabel() == "triangle" ){
                            position = 2;
                        } else position = 1;

                    }
                    telemetry.update();
                }
            }
        }  // end of while

        if(!running) requestOpModeStop();   // user requested to abort setup

        runtime.reset();
        runState = State.TEST;  //Change to State.LEVEL_ADJUST; for normal

        while (opModeIsActive() && (running)) {
            switch(runState){
                case TEST:


                    drive.closeClaw();
                    sleep(400);
                    drive.liftLowJunction();
                    sleep(700);
                    drive.detectJunction(0.2, 2);

                    /*
                    drive.setDrivePower(0.2, .2, .2, .2);
                    boolean flag = false;
                    runtime.reset();
                    while(!flag && (runtime.time() < 3)) {
                        telemetry.addData("sensor Junction", String.format("%.01f in", robot.sensorJunction.getDistance(DistanceUnit.INCH)));
                        telemetry.update();
                        if (robot.sensorJunction.getDistance(DistanceUnit.INCH) < 9) flag = true;
                        if (robot.sensorJunction2.getDistance(DistanceUnit.INCH) < 9) flag = true;
                     }
                    drive.motorsHalt();



                    if (flag) {
                        telemetry.addData("Junction ", "Detected");
                    } else {
                        telemetry.addData("Junction ", "NOT Detected");
                    }
                    telemetry.update();


                     */
                 //   drive.driveDistance(1, 0, 12);
                 //   drive.openClaw();
                 //   sleep(5000);

                    runState = State.HALT;
                    break;

                case LEVEL_ADJUST:
                    robot.lampRobot.setPower(0);
                    runState = State.HIGH_JUNCTION_1;
                    break;

                case HIGH_JUNCTION_1:
                    // starting from start position, close claw
                    drive.closeClaw();
                    //sleep(300);

                    // Drive forward away from wall, pushing signal cone out of position
                    drive.driveDistance(0.8, 0, 61);

                    // raise the arm to position the cone
                    drive.liftHighJunction();

                    //back up to position to score cone
                    drive.driveDistance(0.4,180,4);

                    //turn to high junction
                    drive.PIDRotate(-45,robot.PID_ROTATE_ERROR);
                    drive.PIDRotate(-45,robot.PID_ROTATE_ERROR);

                    //Wait for raise
                    sleep(400);

                    // Drive forward to the high junction
                    drive.driveDistance(0.3,0,4);

                    drive.setDrivePower(0.2, .2, .2, .2);
                    runtime.reset();
                    while((robot.sensorJunction.getDistance(DistanceUnit.INCH) > 9) && (runtime.time() < 2)) {
                        telemetry.addData("sensor Junction", String.format("%.01f in", robot.sensorJunction.getDistance(DistanceUnit.INCH)));
                        telemetry.update();
                    }

                    drive.motorsHalt();

                    drive.driveDistance(0.2, 180, 0.0);

                    // lower the arm and release the cone
                    drive.liftMidJunction();
                    sleep(300);

                    drive.openClaw();

                    // raise the lift to keep from entagling on junction
                    drive.liftHighJunction();
                    sleep(300);
                    drive.liftReset();
                    // back away from the junction
                    drive.driveDistance(0.3, 180, 5);

                    //rotate towards the cone stack
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);

                    // reset the lift to its starting position
                    drive.liftReset();

                    runState = State.CONE_2;
                    break;

                case CONE_2:
                    //rotate towards the cone stack
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);

                    // lower the arm to pick up the top cone
                    drive.liftPosition(robot.LIFT_CONE5);

                    //drive towards the stack of cones
                    drive.driveDistance(0.7,0,16);

                    // adjust direction - turn towards cone stack
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);

                    //drive towards the stack of cones
                    drive.driveDistance(0.4,0,13);

                    // close the claw to grab the cone
                    drive.closeClaw();
                    sleep(700);

                    //back away from the wall slightly
                    drive.driveDistance(0.2,180,0.5);

                    // lift the cone up to clear the stack
                    drive.liftLowJunction();
                    sleep(600);

                    runState = State.LOW_JUNCTION_2;
                    break;

                case LOW_JUNCTION_2:
                    // back away to tile 2
                    drive.driveDistance(0.4,180,23);

                    // rotate towards the low junction
                    drive.PIDRotate(135, robot.PID_ROTATE_ERROR);
                    drive.PIDRotate(135, robot.PID_ROTATE_ERROR);

                    // drive towards the junction
                    drive.driveDistance(0.3, 0, 4);

                    //Turn on drive to sensor
                    drive.setDrivePower(0.2, .2, .2, .2);
                    runtime.reset();
                    while((robot.sensorJunction.getDistance(DistanceUnit.INCH) > 9) && (runtime.time() < 2)) {
                        telemetry.addData("sensor Junction", String.format("%.01f in", robot.sensorJunction.getDistance(DistanceUnit.INCH)));
                        telemetry.update();
                    }

                    drive.motorsHalt();
                    //Overshoot Correct
                    //drive.driveDistance(0.2, 180, 1);


                    // place the cone
                    drive.liftPosition(robot.LIFT_RESET);
                    sleep(400);
                    drive.openClaw();

                    // raise the lift to clear the junction
                    drive.liftLowJunction();
                    sleep(300);

                    // back away from the junction
                    drive.driveDistance(0.3, 180, 9);

                    // turn towards the stack
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);

                    runState = State.CONE_3;
                    break;

                case CONE_3:
                    // lower the arm to pick up the top cone
                    drive.liftPosition(robot.LIFT_CONE4);

                    //drive towards the stack of cones
                    drive.driveDistance(0.6,0,15);

                    // adjust direction - turn towards cone stack
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);

                    //drive towards the stack of cones
                    drive.driveDistance(0.4,0,12);

                    // close the claw to grab the cone

                    drive.closeClaw();
                    sleep(600);

                    //back away from the wall slightly
                    drive.driveDistance(0.2,180,0.5);

                    // lift the cone up to clear the stack
                    drive.liftLowJunction();
                    sleep(600);

                    // back away to tile 2
                    drive.driveDistance(0.4,180,27);

                    runState = State.MID_JUNCTION_3;
                    break;

                case LOW_JUNCTION_3:
                    // rotate towards the low junction
                    drive.PIDRotate(120, robot.PID_ROTATE_ERROR);
                    drive.PIDRotate(120, robot.PID_ROTATE_ERROR);

                    // drive towards the junction
                    drive.driveDistance(0.3, 0, 3);

                    // place the cone
                    drive.liftReset();
                    sleep(400);
                    drive.openClaw();

                    // raise the lift to clear the junction
                    drive.liftLowJunction();

                    // turn towards the stack
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);
                    drive.PIDRotate(90, robot.PID_ROTATE_ERROR);

                    runState = State.PARK;
                    break;

                case MID_JUNCTION_3:
                    // rotate towards the low junction
                    drive.PIDRotate(225, robot.PID_ROTATE_ERROR);
                    drive.PIDRotate(225, robot.PID_ROTATE_ERROR);

                    // raise the arm to position the cone
                    drive.liftMidJunction();

                    //Wait for raise
                    sleep(500);

                    // Drive forward to the high junction
                    drive.driveDistance(0.3,0,3);
                    //Turn on drive to sensor
                    drive.setDrivePower(0.2, .2, .2, .2);
                    runtime.reset();
                    while((robot.sensorJunction.getDistance(DistanceUnit.INCH) > 9) && (runtime.time() < 2)) {
                        telemetry.addData("sensor Junction", String.format("%.01f in", robot.sensorJunction.getDistance(DistanceUnit.INCH)));
                        telemetry.update();
                    }

                    drive.motorsHalt();
                    //Overshoot Correct
                    drive.driveDistance(0.2, 180, 0);
                    // lower the arm and release the cone
                    drive.liftLowJunction();
                    sleep(400);

                    drive.openClaw();

                    // raise the lift to keep from entagling on junction
                    drive.liftMidJunction();

                    // back away from the junction
                    drive.driveDistance(0.3, 180, 5);

                    //rotate towards the cone stack
                    drive.PIDRotate(-90, robot.PID_ROTATE_ERROR);
                    drive.PIDRotate(-90, robot.PID_ROTATE_ERROR);

                    // reset the lift to its starting position
                    drive.liftReset();

                    // back away to center
                    //drive.driveDistance(0.4,0,1.5);

                    runState = State.PARK;
                    break;

                case PARK:

                    if(position == 3) {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the stack to park - ready to grab the first cone in teleop
                        //drive.PIDRotate(90, robot.PID_ROTATE_ERROR);

                        // drive to park position 1
                        drive.driveDistance(0.6, 180,26);

                    } else if (position == 2) {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the outside wall position
                        //drive.PIDRotate(90, robot.PID_ROTATE_ERROR);

                        // drive to park position 1
                        drive.driveDistance(0.3, 180,1);

                    } else {
                        // reset the lift
                        drive.liftReset();
                        drive.openClaw();

                        // rotate towards the outside wall position
                        //drive.PIDRotate(90, robot.PID_ROTATE_ERROR);

                        // drive to park position 1
                        drive.driveDistance(0.6, 0,18);
                    }

                    while(opModeIsActive() && robot.motorBase.getCurrentPosition() > 10){
                        drive.liftReset();
                    }

                    runState = State.HALT;

                    break;


                case HALT:

                    // shut down all motors
                    drive.motorsHalt();

                    running = false;        // exit the program loop
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

