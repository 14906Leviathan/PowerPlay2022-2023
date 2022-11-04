/*
 * Program Name:
 * Alliance:
 * Starting position
 * Functions of the program:
 *  - STEP1 =   gets the foundation into the build site
 *  - STEP2
 *
 *
 *
 */

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.libs.DriveMecanum;

@TeleOp(name = "Broken Bot Testing", group = "Programming Class")
//@Disabled

public class BrokenBot extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    public BrokenBot(){

    }   // end of BrokenBotTS constructor

    public void runOpMode(){
        double startTime;
        double timeElapsed;
        double v1, v2, v3, v4, robotAngle, powerLevel=1;
        double modePower = 1;
        double theta=0;
        double dpadup, dpaddown, dpadleft, dpadright;
        double r;
        double rightX, rightY;
        double peakPower = 0.30;
        boolean fieldCentric = true;
        //double armPosition = 0.1;
        double linearServoPosition = 0.5;

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);

        /*
         * Initialize the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode);

        /*
         * Calibrate / initialize the gyro sensor
         */

        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            /*
             * Mecanum Drive Control section
             */
            if (fieldCentric) {             // verify that the user hasn't disabled field centric drive
//                theta = robot.imu.getAngularOrientation().firstAngle;
            } else {
//                theta = 0;      // do not adjust for the angular position of the robot
            }
            robotAngle = Math.atan2(gamepad1.left_stick_y, (-gamepad1.left_stick_x)) - Math.PI / 4;
            rightX = gamepad1.right_stick_x;
            rightY = -gamepad1.right_stick_y;
            r = -Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta)) + rightX + rightY) * powerLevel;
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta)) - rightX + rightY) * powerLevel;
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta)) + rightX + rightY) * powerLevel;
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta)) - rightX + rightY) * powerLevel;


            if (gamepad1.dpad_down) {
                telemetry.addData("MotorLR = ", "ON");
                dpaddown = 1;
            } else dpaddown = 0;

            if (gamepad1.dpad_up) {
                telemetry.addData("MotorLF = ", "ON");
                dpadup = 1;
            } else dpadup = 0;

            if (gamepad1.dpad_right) {
                telemetry.addData("MotorRR = ", "ON");
                dpadright = 1;
            } else dpadright = 0;

            if (gamepad1.dpad_left) {
                telemetry.addData("MotorRF = ", "ON");
                dpadleft = 1;
            } else dpadleft = 0;

            //  if(gamepad2.dpad_up){
            //      linearServoPosition = linearServoPosition + 0.01;
            //      sleep(200);
            //  } else if(gamepad2.dpad_down){
            //      linearServoPosition = linearServoPosition - 0.01;
            //      sleep(200);
            //  }

            //    robot.servoLinear.setPosition(linearServoPosition);

            robot.motorLF.setPower(v1 * modePower + dpadup);
            robot.motorRF.setPower(v2 * modePower + dpadleft);
            robot.motorLR.setPower(v3 * modePower + dpaddown);
            robot.motorRR.setPower(v4 * modePower + dpadright);

            telemetry.addData("Servo Position = ", robot.motorArm.getCurrentPosition());
            telemetry.addData("motorLF = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("motorLR = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("motorRF = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("motorRR = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("Shooter Encoder = ", robot.motorTurnTable.getCurrentPosition());
            //telemetry.addData("IMU Value: ", theta);
            telemetry.update();

            if (gamepad1.a) {
                robot.motorArm.setPower(0.5);
            } else if (gamepad1.b) {
                robot.motorArm.setPower(-0.5);
            } else robot.motorArm.setPower(0);

            if (gamepad1.x){
                robot.motorBase.setPower(0.5);
            } else if (gamepad1.y) {
                robot.motorBase.setPower(-0.5);
            } else robot.motorBase.setPower(0);



        }   // end of while opModeIsActive()

    }   // end of runOpMode method
}   // end of BrokenBotTS.java class
