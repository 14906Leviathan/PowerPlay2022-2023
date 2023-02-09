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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.libs.DriveMecanum;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleoOpRuntoP", group = "Comp")
//@Disabled

public class TeloOpRuntoP extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;
    private ElapsedTime elapsedTime = new ElapsedTime();
    public TeloOpRuntoP(){

    }   // end of BrokenBotTS constructor

    public void runOpMode() {
        double v1, v2, v3, v4, robotAngle;
        double modePower = 1;
        double theta = 0;
        int mBase = 0;
        double r;
        double rightX, rightY;
        boolean fieldCentric = false;
        double delay=elapsedTime.time();

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        /*
         * Setup the initial state of the robot
         */
        robot.init(hardwareMap);
        robot.motorBase.setPower(1.0);

        /*
         * Initialize the drive class
         */
        DriveMecanum drive = new DriveMecanum(robot, opMode);

        /*
         * Calibrate / initialize the gyro sensor
         */

        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Greetings = ", "HOME CHICKEN");
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

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
            v1 = (r * Math.cos(robotAngle - Math.toRadians(theta)) + rightX + rightY);
            v2 = (r * Math.sin(robotAngle - Math.toRadians(theta)) - rightX + rightY);
            v3 = (r * Math.sin(robotAngle - Math.toRadians(theta)) + rightX + rightY);
            v4 = (r * Math.cos(robotAngle - Math.toRadians(theta)) - rightX + rightY);

            robot.motorLF.setPower(v1 * modePower);
            robot.motorRF.setPower(v2 * modePower);
            robot.motorLR.setPower(v3 * modePower);
            robot.motorRR.setPower(v4 * modePower);

            /* #################################################################################
               ####         Claw Control
             * #################################################################################*/

            if (gamepad1.right_trigger > 0.1) {
                robot.servoGrabber.setPosition(robot.SERVO_GRAB_OPEN);
            } else if (gamepad1.left_trigger > 0.1) {
                robot.servoGrabber.setPosition(robot.SERVO_GRAB_CLOSE);
            }

            /* #################################################################################
               ####         Lift Control
             * #################################################################################*/
            if(gamepad1.x){
                // Set to low junction level
                mBase = robot.LIFT_LOW_JUNCTION;
            }   // end of if(gamepad1.x)

            if(gamepad1.b){
                // set to mid junction level
                mBase = robot.LIFT_MID_JUNCTION;
            }   // end of if(gamepad1.b)

            if(gamepad1.y){
                // set to high junction
                mBase = robot.LIFT_HIGH_JUNCTION;
            }   // end of if(gamepad1.y)

            if(gamepad1.a){

                if (mBase != 0){
                    // reset lift to lowest position
                    mBase = robot.LIFT_RESET;


                }
                else if(robot.motorBase.getCurrentPosition()<10) {
                    // lift to cone f
                    mBase = robot.LIFT_CONE5;
                }
            }   // end of if(gamepad1.a)

            // allow manual control of the lift
            if (gamepad1.dpad_up) {
                mBase+=15;
            }else if (gamepad1.dpad_down) {
                mBase-=15;
            }   // end of if(gamepad1.dpad_up)

            // limit the max and min value of mBase
            Range.clip(mBase, robot.LIFT_MIN_LOW,robot.LIFT_MAX_HIGH);
            drive.liftPosition(mBase);

            if(gamepad1.dpad_left){

                drive.PIDRotate((drive.getZAngle() + 180),5);
                                }

            if(gamepad1.dpad_right){

                drive.PIDRotate((drive.getZAngle() - 180),5);
            }

            telemetry.addData("motorLF = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("motorLR = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("motorRF = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("motorRR = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("motorBase = ", robot.motorBase.getCurrentPosition());
            telemetry.addData("motorBase power = ",robot.motorBase.getPower());
            telemetry.addData("Base Setpoint = ", mBase);
            //telemetry.addData("IMU Value: ", theta);
            telemetry.update();

        }   // end of while opModeIsActive()

    }   // end of runOpMode method
}   // end of Linear op mode