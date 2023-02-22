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

@Autonomous(name = "Auto TEST - Tuning", group = "Leviathan")

public class AutoRobotTuning extends LinearOpMode {

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
    private State runState = State.TEST;
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    int position = 3;

    public AutoRobotTuning(){

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
        runState = State.TEST;

        while (opModeIsActive()) {
            switch(runState){
                case TEST:
                    /*
                    drive.ftclibDrive(45, 16, 0);
                    drive.ftclibDrive(225, 16, 0);
                    drive.ftclibDrive(-45, 8, 0);
                    drive.ftclibDrive(135, 8, 0);
                    */

                    telemetry.addData("rotating : ", "45 degrees");
                    telemetry.update();
                    drive.ftclibRotate(45, 1);
                    sleep(2000);

                    telemetry.addData("rotating : ", "0 degrees");
                    telemetry.update();
                    drive.ftclibRotate(0, 1);
                    sleep(2000);

                    telemetry.addData("rotating : ", "-45 degrees");
                    telemetry.update();
                    drive.ftclibRotate(-45, 1);
                    sleep(2000);

                    telemetry.addData("rotating : ", "0 degrees");
                    telemetry.update();
                    drive.ftclibRotate(0, 1);
                    sleep(2000);

                    telemetry.addData("rotating : ", "90 degrees");
                    telemetry.update();
                    drive.ftclibRotate(90, 1);
                    sleep(2000);

                    telemetry.addData("rotating : ", "0 degrees");
                    telemetry.update();
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

                case HALT:

                    // shut down all motors
                    drive.motorsHalt();

                    requestOpModeStop();    // request stoppage of the program

                    break;
            }   // end of switch(state)
        }   // end of while(opModeIsActive)

        requestOpModeStop();

    } // end of opmode

    /*
     * Enumerate the states of the machine
     */
    enum State {
        TEST, HIGH_JUNCTION_1, HALT
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

