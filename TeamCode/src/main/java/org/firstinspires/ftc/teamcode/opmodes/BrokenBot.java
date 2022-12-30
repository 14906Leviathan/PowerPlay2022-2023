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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.HardwareProfile;
import org.firstinspires.ftc.teamcode.libs.DriveMecanum;

@Config
@TeleOp(name = "Broken Bot", group = "Test")
//@Disabled

public class BrokenBot extends LinearOpMode {

    private final static HardwareProfile robot = new HardwareProfile();
    private LinearOpMode opMode = this;

    FtcDashboard dashboard;
    public static double l1_CLAW_OPEN = robot.SERVO_GRAB_OPEN;
    public static double l2_CLAW_CLOSE = robot.SERVO_GRAB_CLOSE;
    public static int l3_LIFT_JUNCTION_HIGH = robot.LIFT_HIGH_JUNCTION;
    public static int l4_LIFT_JUNCTION_MID = robot.LIFT_MID_JUNCTION;
    public static int l5_LIFT_JUNCTION_LOW = robot.LIFT_LOW_JUNCTION;
    public static int l6_LIFT_POSITION = 0;

    public BrokenBot(){

    }   // end of BrokenBot constructor

    public void runOpMode() {
        double v1, v2, v3, v4, robotAngle;
        double modePower = 1;
        double theta = 0;
        int mBase = 0;
        double r;
        double rightX, rightY;
        double spinpower = 0;
        boolean fieldCentric = true;

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();


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
         * Ready to go
         */

        // post telemetry to FTC Dashboard as well
        dashTelemetry.put("01 - IMU Angle X = ", robot.imu.getAngularOrientation().firstAngle);
        dashTelemetry.put("02 - IMU Angle Y = ", robot.imu.getAngularOrientation().secondAngle);
        dashTelemetry.put("03 - IMU Angle Z = ", robot.imu.getAngularOrientation().thirdAngle);
        dashTelemetry.put("04 - Lift Front Encoder Value = ", robot.motorBase.getCurrentPosition());
        dashTelemetry.put("06 - Claw Value = ", robot.servoGrabber.getPosition());
        dashTelemetry.put("07 - GP1.Button.X = ", "RESET LIFT");
        dashTelemetry.put("08 - GP1.Button.A = ", "LIFT LOW JUNCTION");
        dashTelemetry.put("09 - GP1.Button.B = ", "LIFT MID JUNCTION");
        dashTelemetry.put("10 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
        dashTelemetry.put("11 - GP2.Button.A = ", "Custom Position - program stack cone levels");
        dashboard.sendTelemetryPacket(dashTelemetry);

        telemetry.addData("Z Value = ", drive.getZAngle());
        telemetry.addData("Greetings = ", "HOME CHICKEN");
        telemetry.addData("Robot state = ", "INITIALIZED");
        telemetry.update();

        waitForStart();

        int rfStart = robot.motorRF.getCurrentPosition();
        int lfStart = robot.motorLF.getCurrentPosition();
        int rrStart = robot.motorRR.getCurrentPosition();
        int lrStart = robot.motorLR.getCurrentPosition();

        while (opModeIsActive()) {

            /* #################################################################################
               ####         Mecanum Drive Control section
             * #################################################################################*/

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

            if(gamepad2.dpad_up) {
                v1 = 1;
            } else if (gamepad2.dpad_down) {
                v3 = 1;
            } else if (gamepad2.dpad_left) {
                v2 = 1;
            } else if (gamepad2.dpad_right){
                v4 = 1;
            }

            robot.motorLF.setPower(v1 * modePower);
            robot.motorRF.setPower(v2 * modePower);
            robot.motorLR.setPower(v3 * modePower);
            robot.motorRR.setPower(v4 * modePower);

            /* #################################################################################
               ####         Lift Control
             * #################################################################################*/
            if(gamepad1.a){
                // Set to low junction level
                mBase = l5_LIFT_JUNCTION_LOW;
            }   // end of if(gamepad1.a)

            if(gamepad1.b){
                // set to mid junction level
                mBase = l4_LIFT_JUNCTION_MID;
            }   // end of if(gamepad1.b)

            if(gamepad1.y){
                // set to high junction
                mBase = l3_LIFT_JUNCTION_HIGH;
            }   // end of if(gamepad1.y)

            if(gamepad1.x){
                // reset lift to lowest position
                mBase = robot.LIFT_RESET;
            }   // end of if(gamepad1.x)

            if(gamepad2.a){
                // Set to low junction level
                mBase = l6_LIFT_POSITION;
            }   // end of if(gamepad2.a)

            // limit the values of liftPosition => This shouldn't be necessary if logic above works
            // allow manual control of the lift
            if (gamepad1.dpad_up) {
                mBase+=15;
            }else if (gamepad1.dpad_down) {
                mBase-=15;
            }   // end of if(gamepad1.dpad_up)

            // limit the max and min value of mBase
            Range.clip(mBase, robot.LIFT_MIN_LOW,robot.LIFT_MAX_HIGH);
            drive.liftPosition(mBase);

            /* #################################################################################
               ####         Claw Control
             * #################################################################################*/

            if (gamepad1.right_trigger > 0.1) {
                drive.openClaw();
            } else if (gamepad1.left_trigger > 0.1) {
                drive.closeClaw();
            }   // end of if(gamepad1.right_trigger)


            /* #################################################################################
               ####         User Feedback
             * #################################################################################*/

            telemetry.addData("Gyro Value = ", drive.getZAngle());
            telemetry.addData("LF Start= ", lfStart);
            telemetry.addData("LR Start= ", lrStart);
            telemetry.addData("RF Start= ", rfStart);
            telemetry.addData("RR Start= ", rrStart);
            telemetry.addData("    ", "");
            telemetry.addData("Drive Distance = ", drive.calcDistance(0, rfStart, rrStart, lfStart, lrStart));
            telemetry.addData("motorLF = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("motorLR = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("motorRF = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("motorRR = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("motorBase = ", robot.motorBase.getCurrentPosition());
            if(v1 > 0) {
                telemetry.addData("Motor Left Front = ", v1);
            }
            if (v2 > 0) {
                telemetry.addData("Motor Right Front = ", v2);
            }
            if (v3 > 0) {
                telemetry.addData("Motor Left Rear = ", v3);
            }
            if (v4 > 0) {
                telemetry.addData("Motor Right Rear = ", v4);
            }
            telemetry.addData("motorBase power = ",robot.motorBase.getPower());
            telemetry.addData("spinpower = ", spinpower);
            telemetry.addData("Base Setpoint = ", mBase);
            telemetry.update();

            // post telemetry to FTC Dashboard as well
            dashTelemetry.put("01 - IMU Angle X = ", robot.imu.getAngularOrientation().firstAngle);
            dashTelemetry.put("02 - IMU Angle Y = ", robot.imu.getAngularOrientation().secondAngle);
            dashTelemetry.put("03 - IMU Angle Z = ", robot.imu.getAngularOrientation().thirdAngle);
            dashTelemetry.put("04 - Lift Front Encoder Value = ", robot.motorBase.getCurrentPosition());
            dashTelemetry.put("06 - Claw Value = ", robot.servoGrabber.getPosition());
            dashTelemetry.put("07 - GP1.Button.X = ", "RESET LIFT");
            dashTelemetry.put("08 - GP1.Button.A = ", "LIFT LOW JUNCTION");
            dashTelemetry.put("09 - GP1.Button.B = ", "LIFT MID JUNCTION");
            dashTelemetry.put("10 - GP1.Button.Y = ", "LIFT HIGH JUNCTION");
            dashTelemetry.put("11 - GP2.Button.A = ", "Custom Position - program stack cone levels");
            dashboard.sendTelemetryPacket(dashTelemetry);

        }   // end of while opModeIsActive()

    }   // end of runOpMode method
}   // end of Linear op mode
