package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.libs.DriveMecanum;


import java.util.List;

@Autonomous(name = "Remote Auto - LevBlueDuck", group = "Leviathan")

public class LevBlueDuck extends LinearOpMode{

    public static final String TFOD_MODEL_ASSET = "PP_FF_TSEv3-Green.tflite";
    public static final String[] LABELS = {
            "TSEv3"
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
    boolean debugMode = false;
    int mArm = 0;
    int mBase = 0;
    /* Declare DataLogger variables */
    private String action = "";

    @Override
    public void runOpMode() {

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
        robot.motorBase.setTargetPosition(0);
        //robot.motorArm.setTargetPosition(0);
        robot.motorBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // monitor the position of the TSE on the field
        while(!opModeIsActive() && running){
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if (recognition.getLeft() < 150) {
                            scoreLevel = 1;
                        } else if (recognition.getLeft() > 150 && recognition.getLeft() < 400 ) {
                            scoreLevel = 2;
                        } else if (recognition.getLeft() > 400) {
                            scoreLevel = 3;
                        }
                        i++;
                        if (updatedRecognitions.size() == 0) scoreLevel = 1;
                    }     // if (Recognition...
                    // Send telemetry message to signify robot waiting;
                    telemetry.addData("Robot Status : ", "READY TO RUN");    //
                    telemetry.addData("Scanning for : ", "TSE");
                    if (scoreLevel == 1){
                        telemetry.addData("Detected Level = ","Bottom");
                    }else if (scoreLevel == 2){
                        telemetry.addData("Detected Level = ","Middle");
                    } else{
                        telemetry.addData("Detected Level = ","Top");
                    }
                    telemetry.addData("Press X to : ", "ABORT Program");
                    telemetry.update();
                }   // if (updatedRecog...)
            }   // end of if (tfod != null)

            if(gamepad1.x || gamepad2.x) running = false;   // abort the program
        }   // end of while(!opModeIsActive...

        if(!running) requestOpModeStop();   // user requested to abort setup

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();
        runState = State.LEVEL_ADJUST;

        while (opModeIsActive() && (running)) {
            switch(runState){
                case TEST:
                    runState = State.HALT;
                    break;

                case LEVEL_ADJUST:

                    if(scoreLevel == 1){
                        // set arm position to level 1
                        mArm=620;
                        mBase=-640;
                    } else if(scoreLevel == 2) {
                        // set arm position to level 2
                        mArm=760;
                        mBase=-520;
                    } else {
                        // set arm position to level 3
                        mArm=1250;
                        mBase=-800;
                    }

                    telemetry.addData("TSE Position = ", scoreLevel);
                    telemetry.update();
                    //runState = State.HALT;  //FOR POSITION TEST
                    runState = State.RUN;
                    break;

                case RUN:
                    // Drive off wall
                    drive.robotCorrect(0.5, 0, 0.5);
                    // Rotate 90
                    drive.PIDRotate(90,0.3);
                    // Drive to wall, fast and then slow
                    drive.robotCorrect(0.5, 0, 1.0);
                    drive.robotCorrect(0.25, 0, 0.8);
                    // Strafe into Turn table
                    drive.driveSensorDistance(0.25, 90, 6.8);
                    // Drive back to the wall at a very slow rate
                    drive.robotCorrect(0.1,0,0.1);


                    // Run turn table motor, drop duck
                    //robot.motorTurnTable.setPower(0.2);

                    sleep(4000);

                    //robot.motorTurnTable.setPower(0);

                    drive.robotCorrect(0.3,180,0.1);

                    // Strafe into parking spot
                    drive.driveSensorDistanceOut(0.25, -90, 26);
                   //line up for TAH
                    drive.driveSensorDistanceOut(0.25, -90, 42);
                    // Drive off wall
                    drive.robotCorrect(0.5, 0, 0.5);

                    drive.PIDRotate(-90,0.3);
                    drive.PIDRotate(-90,0.3);



                    //robot.motorArm.setPower(0.40);
                    //robot.motorArm.setTargetPosition(mArm);
                    robot.motorBase.setPower(0.40);
                    robot.motorBase.setTargetPosition(mBase);
                    drive.robotCorrect(0.5, 180, 0.5);
                    drive.robotCorrect(0.5, 0, 0.5);
                    // Drive off wall
                    //drive.robotCorrect(0.5, 0, 0.5);
                    // Rotate 45
                    //drive.PIDRotate(-45,0.3);
                    // Drive to SE, fast and then slow
                    drive.robotCorrect(0.5, 0, 0.6);
                    drive.robotCorrect(0.25, 0, 0.25);
                    // Drop off block
                    robot.servoRwheel.setPosition(1);
                    drive.robotCorrect(0.01, 0, 0.9);
                    robot.servoLwheel.setPosition(0);
                    //lower arm
                    mArm=0;
                    mBase=0;
                    //robot.motorArm.setPower(0.40);
                    //robot.motorArm.setTargetPosition(mArm);
                    robot.motorBase.setPower(0.40);
                    robot.motorBase.setTargetPosition(mBase);
                    sleep(2000);
                    //reverse to wall
                    drive.robotCorrect(0.5, 180, 1.25);
                    // Rotate 90
                    drive.PIDRotate(90,0.3);

                    drive.robotCorrect(0.5, 0, 0.5);
                    //slide to wall
                    drive.driveSensorDistance(0.25, 90, 26);
                    //drive to Whse
                    drive.robotCorrect(0.5, 0, 0.5);

                    drive.robotCorrect(0.01, 0, 0.9);
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
        TEST, ALLIANCE_SELECT, RUN,TEST_CONFIG, LEVEL_ADJUST, HALT, SET_DISTANCES
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
