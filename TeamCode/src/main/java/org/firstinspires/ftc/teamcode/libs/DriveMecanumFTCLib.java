package org.firstinspires.ftc.teamcode.libs;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import android.os.BadParcelableException;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.HardwareProfileFTClib;

public class DriveMecanumFTCLib {

    private HardwareProfileFTClib robot;
    public double RF, LF, LR, RR;
    public LinearOpMode opMode;

    FtcDashboard dashboard;

    /*
     * Constructor method
     */
    public DriveMecanumFTCLib(HardwareProfileFTClib myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close DriveMecanum constructor Method


    /* #########################################################################################
       #########################################################################################
       ################################  DRIVE METHODS #########################################
       #########################################################################################
       #########################################################################################
     */

    /******************************************************************************************
     * Method:      driveDistance
     * Function:    Robot drives the direction of the heading, at the power provided,
     *              for the distance provided
     * Note:        This function is intended to work at 0, 90, 180, and -90 headings
     * Parameters:
     * @param power     - defines motor power to be applied
     * @param heading   - Direction robot should drive
     * @param distance  - Distance in Inches to drive
     */
    public void driveDistance(double power, double heading, double distance) {
        double initZ = getZAngle();
        double currentZ, zCorrection, distanceTraveled;
        boolean active = true;

        double theta = Math.toRadians(90 + heading);
        robot.motorLF.resetEncoder();
        robot.motorLR.resetEncoder();
        robot.motorRF.resetEncoder();
        robot.motorRR.resetEncoder();

        while(opMode.opModeIsActive() && active) {

            RF = power * (Math.sin(theta) - Math.cos(theta));
            LF = power * (Math.sin(theta) + Math.cos(theta));
            LR = power * (Math.sin(theta) - Math.cos(theta));
            RR = power * (Math.sin(theta) + Math.cos(theta));

            if (initZ > 170 || initZ < -170){
                currentZ = gyro360(0);      // always use 0 as the reference angle
            } else {
                currentZ = getZAngle();
            }
            if (currentZ != initZ){
                zCorrection = Math.abs(initZ - currentZ)/100;

                if (initZ < currentZ) {
                    RF = RF + zCorrection;
                    RR = RR + zCorrection;
                    LF = LF - zCorrection;
                    LR = LR - zCorrection;
                }
                if (initZ > currentZ) {
                    RF = RF - zCorrection;
                    RR = RR - zCorrection;
                    LF = LF + zCorrection;
                    LR = LR + zCorrection;
                }
            }   // end of if currentZ != initZ

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            RF = Range.clip(RF, -power,power);
            LF = Range.clip(LF, -power,power);
            RR = Range.clip(RR, -power,power);
            LR = Range.clip(LR, -power,power);

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);

            distanceTraveled = calcDistance(heading);
            if(distanceTraveled >= distance) active = false;
            opMode.telemetry.addData("Distance Traveled = ", distanceTraveled);
            opMode.telemetry.addData("Distance = ", distance);
            opMode.telemetry.update();
            opMode.idle();

        }   // end of while loop
        motorsHalt();
    }   // close driveDistance method

    /******************************************************************************************
     * Method:      ftclibDrive
     * Function:    Robot drives the direction of the heading, at the power provided,
     *              for the distance provided
     * Note:        This function is intended to work at 0, 90, 180, and -90 headings
     * Parameters:
     * @param heading   - Direction robot should drive
     * @param distance  - Distance in Inches to drive
     */
    public void ftclibDrive(double heading, double distance, double timeout) {
        dashboard = FtcDashboard.getInstance();
        ElapsedTime overtime = new ElapsedTime();
        TelemetryPacket dashTelemetry = new TelemetryPacket();
        double initZ = getZAngle();
        double rflrPower = 0;
        double lfrrPower = 0;
        double currentZ, zCorrection, distanceTraveled = 0;
        boolean active = true; boolean correct = true;
        double derivative = 0, lastError=0, error=0;
        double integral = 0, drivePower = 0;
        ElapsedTime rotateTime = new ElapsedTime();
        double maxDrivePower = 0.4;
        double Kp = 0.2;
        double Ki = 0.001;
        double Kd = 0.01;
        double minSpeed = 0.15;
        double theta = Math.toRadians(90 + heading);

        robot.motorLF.resetEncoder();
        robot.motorLR.resetEncoder();
        robot.motorRF.resetEncoder();
        robot.motorRR.resetEncoder();

        opMode.sleep(100);  // allow time for encoder resets
        // make sure distance is positive. Use heading to change direction.
        distance = Math.abs(distance);

        overtime.reset();

        while (opMode.opModeIsActive() && active) {

            error = distance - distanceTraveled;
            derivative = lastError - error;
            integral = rotateTime.time() * error;
            drivePower = ((Kp * error) + (Ki * integral) + (Kd * derivative));
            lastError = error;


            if (drivePower > -0.10 && drivePower < 0 ){
                drivePower = minSpeed;
            } else if (drivePower <0.10 && drivePower > 0){
                drivePower = minSpeed;
            }
            rflrPower = drivePower * (Math.sin(theta) - Math.cos(theta));
            lfrrPower = drivePower * (Math.sin(theta) + Math.cos(theta));

            if (initZ > 170 || initZ < -170) {
                currentZ = gyro360(0);      // always use 0 as the reference angle
            } else {
                currentZ = getZAngle();
            }
            if (currentZ != initZ) {
                zCorrection = Math.abs(initZ - currentZ) / 100;

                if (initZ < currentZ) {
                    rflrPower = rflrPower + zCorrection;
                    lfrrPower = lfrrPower - zCorrection;
                }
                if (initZ > currentZ) {
                    rflrPower = rflrPower - zCorrection;
                    lfrrPower = lfrrPower + zCorrection;
                }
            }   // end of if currentZ != initZ

            /*
             * Limit that value of the drive motors so that the power does not exceed 100%
             */
            rflrPower = Range.clip(rflrPower, -Math.abs(maxDrivePower), Math.abs(maxDrivePower));
            lfrrPower = Range.clip(lfrrPower, -Math.abs(maxDrivePower), Math.abs(maxDrivePower));

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(rflrPower, lfrrPower, rflrPower, lfrrPower);

            distanceTraveled = calcDistance(heading);
            if (distanceTraveled >= distance) {
                active = false;
            }
            if(timeout != 0){
                if(overtime.time() > timeout){
                    active = false;
                }
            }
            /*
            opMode.telemetry.addData("Distance Traveled = ", distanceTraveled);
            opMode.telemetry.addData("Distance = ", distance);
            opMode.telemetry.update();
            opMode.idle();

            dashTelemetry.put("p20 - drive Telemetry Data", "");
            dashTelemetry.put("p25 - Drive Power                  = ", drivePower);
            dashTelemetry.put("p24 - Target Distance              = ", distance);
            dashTelemetry.put("p21 - rflrPower                    = ", rflrPower);
            dashTelemetry.put("p22 - lfrrPower                    = ", lfrrPower);
            dashTelemetry.put("p26 - Distance Traveled            = ", distanceTraveled);
            dashTelemetry.put("p23 - PID IMU Angle X              = ", robot.imu.getAngles()[0]);
            dashboard.sendTelemetryPacket(dashTelemetry);

             */

        }   // end of while loop
        motorsHalt();

  //      }
    }   // close driveDistance method

    /**
     * Method: PIDRotate
     * Parameters:
     * @param targetAngle -> desire ending angle/position of the robot
     * @param targetError -> how close should the robot get to the desired angle
     */
    public void ftclibRotate(double targetAngle, double targetError){
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        double integral = 0;
        ElapsedTime rotateTime = new ElapsedTime();
        double error;
        double Kp = 0.85;
        double Ki = 0.0; //0.001;
        double Kd = 0.2; //0.02;
        double minRotateSpeed = 0.15;
        double maxRotateSpeed = 1;
        double rotationSpeed;
        double derivative = 0, lastError=0;
        double rightRotate = 0;
        double leftRotate = 0;

        // check to see how far the robot is rotating to decide which gyro sensor value to use

        targetAngle = Math.toRadians(targetAngle);  // convert targetAngle to radians
        error = getZAngleRadians() - targetAngle;
        targetError = Math.toRadians(targetError);

        // reset the time to track rotation speed
        rotateTime.reset();

        while ((Math.abs(error) >= Math.abs(targetError)) && opMode.opModeIsActive()) {
            derivative = (error - lastError) / rotateTime.time();
            integral = integral + (rotateTime.time() * error);
            rotationSpeed = ((Kp * error) + (Ki * integral) + (Kd * derivative));
            lastError = error;

            // Clip motor speed
            rotationSpeed = clamp(rotationSpeed, -maxRotateSpeed, maxRotateSpeed);

            if ((rotationSpeed > -0.10) && (rotationSpeed < 0)) {
                rotationSpeed = -minRotateSpeed;
            } else if ((rotationSpeed < 0.10) && (rotationSpeed > 0)) {
                rotationSpeed = minRotateSpeed;
            }

            leftRotate = -rotationSpeed;
            rightRotate = rotationSpeed;

            setDrivePower(rightRotate, leftRotate, leftRotate, rightRotate);

            // check to see how far the robot is rotating to decide which gyro sensor value to use
            error = getZAngleRadians() - targetAngle;

            dashTelemetry.put("p00 - PIDTurn Telemetry Data", "");
            dashTelemetry.put("p01 - PID IMU Angle X              = ", getZAngle());
            dashTelemetry.put("p02 - PID IMU Angle Y              = ", robot.imu.getAngles()[1]);
            dashTelemetry.put("p03 - PID IMU Angle Z              = ", robot.imu.getAngles()[2]);
            dashTelemetry.put("p04 - targetAngle (Radians)        = ", targetAngle);
            dashTelemetry.put("p05 - Current Angle (Radians)      = ", getZAngleRadians());
            dashTelemetry.put("p06 - Turn Speed                   = ", rotationSpeed);
            dashTelemetry.put("p07 - Angle Error (Radians)        = ", error);
            dashTelemetry.put("p08 - Angle Error (Degrees)        = ", Math.toDegrees(getZAngleRadians()-targetAngle));
            dashTelemetry.put("p09 - derivative                   = ", derivative);
            dashTelemetry.put("p10 - integral                     = ", integral);
            dashTelemetry.put("p11 - Turn Time                    = ", rotateTime.time());
            dashboard.sendTelemetryPacket(dashTelemetry);

        }   // end of while Math.abs(error)
        motorsHalt();

    }   //end of the PIDRotate Method

    /**
     * Method: PIDRotate
     * Parameters:
     * @param targetAngle -> desire ending angle/position of the robot
     * @param targetError -> how close should the robot get to the desired angle
     */
    public void PIDRotate(double targetAngle, double targetError){
        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        double integral = 0;
        ElapsedTime timeElapsed = new ElapsedTime();
        double startTime = timeElapsed.time();
        double totalTime;
        double error;
        double Cp = robot.PID_Kp;
        double Ci = robot.PID_Ki;
        double Cd = robot.PID_Kd;
        /* enable these for tuning
        double Cp = kP;
        double Ci = kI;
        double Cd = kD;
        double maxRotateSpeed = maxSpeed;
        double maxRotateSpeed = minSpeed;
         */
        double minRotateSpeed = robot.PID_MIN_SPEED;
        double maxRotateSpeed = 1;
        double rotationSpeed;
        double derivative = 0, lastError=0;

        // check to see how far the robot is rotating to decide which gyro sensor value to use
        if(targetAngle > 90 || targetAngle < -90){
            error = gyro360(targetAngle) - targetAngle;
        } else {
            error = getZAngle() - targetAngle;
        }

        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
            while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
                derivative = lastError - error;
                rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative));
                lastError = error;

                // Clip motor speed
                rotationSpeed = Range.clip(rotationSpeed, -maxRotateSpeed, maxRotateSpeed);

                if ((rotationSpeed > -0.25) && (rotationSpeed < 0)) {
                    rotationSpeed = -minRotateSpeed;
                } else if ((rotationSpeed < 0.25) && (rotationSpeed > 0)) {
                    rotationSpeed = minRotateSpeed;
                }

                RF = rotationSpeed;
                LF = -rotationSpeed;
                LR = -rotationSpeed;
                RR = rotationSpeed;

                setDrivePower(RF, LF, LR, RR);

                opMode.idle();

                // check to see how far the robot is rotating to decide which gyro sensor value to use
                if (targetAngle > 90 || targetAngle < -90) {
                    error = gyro360(targetAngle) - targetAngle;
                } else {
                    error = getZAngle() - targetAngle;
                }

            }   // end of while Math.abs(error)
            setDrivePower(0,0,0,0);
            maxRotateSpeed = maxRotateSpeed / 2;
            opMode.sleep(20);
//            opMode.idle();

            // Perform a final calc on the error to confirm that the robot didn't overshoot the
            // target position after the last measurement was taken.
//            opMode.sleep(5);
            if (targetAngle > 90 || targetAngle < -90) {
                error = gyro360(targetAngle) - targetAngle;
            } else {
                error = -robot.imu.getAngles()[0] - targetAngle;
//                error = getZAngle() - targetAngle;
            }

            totalTime = timeElapsed.time() - startTime;
            // post telemetry to FTC Dashboard
            dashTelemetry.put("p00 - PIDTurn Telemetry Data", "");
            dashTelemetry.put("p01 - PID IMU Angle X                  = ", robot.imu.getAngles()[0]);
            dashTelemetry.put("p02 - PID IMU Angle Y                  = ", robot.imu.getAngles()[1]);
            dashTelemetry.put("p03 - PID IMU Angle Z                  = ", robot.imu.getAngles()[2]);
            dashTelemetry.put("p04 - InitZ/targetAngle value      = ", targetAngle);
            dashTelemetry.put("p05 - Current Angle                = ", getZAngle());
            dashTelemetry.put("p06 - Angle Error                  = ", error);
            dashTelemetry.put("p07 - zCorrection/derivative Value = ", derivative);
            dashTelemetry.put("p08 - Turn Time                    = ", totalTime);
            dashTelemetry.put("p09 - Right Front                  = ", RF);
            dashTelemetry.put("p10 - Right Rear                   = ", RR);
            dashTelemetry.put("p11 - Left Front                   = ", LF);
            dashTelemetry.put("p12 - Right Rear                   = ", RR);
            dashboard.sendTelemetryPacket(dashTelemetry);

        }

        // shut off the drive motors
        motorsHalt();

    }   //end of the PIDRotate Method


    public void detectJunction(double forwardSpeed, double timeout){
        ElapsedTime localtime = new ElapsedTime();


        setDrivePower(forwardSpeed, forwardSpeed, forwardSpeed, forwardSpeed);
        localtime.reset();
        while(((robot.sensorJunction.getDistance(DistanceUnit.INCH) > 7) &&
                (robot.sensorJunction2.getDistance(DistanceUnit.INCH) > 7)) && (localtime.time() < timeout)) {
            opMode.telemetry.addData("sensor Junction", String.format("%.01f in", robot.sensorJunction.getDistance(DistanceUnit.INCH)));
            opMode.telemetry.update();
        }

        motorsHalt();

    }


    /******************************************************************************************
     * Sets power to all four drive motors
     * Method:
     * @param RF    - power for right front motor
     * @param LF    - power for left front motor
     * @param LR    - power for left rear motor
     * @param RR    - power for right rear motor
     ******************************************************************************************/
    public void setDrivePower(double RF, double LF, double LR, double RR){
        robot.motorRF.set(RF);
        robot.motorLF.set(LF);
        robot.motorLR.set(LR);
        robot.motorRR.set(RR);
    }   // end of setDrivePower method

    /******************************************************************************************
     * Method:      motorsHalt
     * Function:    Shut off all drive motors
     ******************************************************************************************/
    public void motorsHalt(){
        robot.motorRF.stopMotor();
        robot.motorLF.stopMotor();
        robot.motorLR.stopMotor();
        robot.motorRR.stopMotor();
    }   // end of motorsHalt method


    /* #########################################################################################
       #########################################################################################
       ################################  MECHANISM METHODS #####################################
       #########################################################################################
       #########################################################################################
     */

    public void releaseCone(){
        robot.servoAlign.setPosition(robot.SERVO_ALIGN_DOWN);
        opMode.sleep(200);

    }
    public void closeClaw(){
        robot.servoGrabber.setPosition(robot.SERVO_GRAB_CLOSE);
    }   // end of closeClaw method

    public void openClaw(){
        robot.servoGrabber.setPosition(robot.SERVO_GRAB_OPEN);
    }   // end of openClaw method

    public void alignUp(){
        robot.servoAlign.setPosition(robot.SERVO_ALIGN_UP);
    }

    public void alignDown(){
        robot.servoAlign.setPosition(robot.SERVO_ALIGN_DOWN);
    }

    public void liftReset(){
        robot.motorBase.setPower(robot.LIFT_POWER);
        robot.motorBase.setTargetPosition(robot.LIFT_RESET);
    }   // end of liftReset method

    public void liftLowJunction(){
        robot.motorBase.setPower(robot.LIFT_POWER);
        robot.motorBase.setTargetPosition(robot.LIFT_LOW_JUNCTION);
    }   // end of liftLowJunction method

    public void liftMidJunction(){
        robot.motorBase.setPower(robot.LIFT_POWER);
        robot.motorBase.setTargetPosition(robot.LIFT_MID_JUNCTION);
    }   // end of liftMidJunction method

    public void liftHighJunction(){
        robot.motorBase.setPower(robot.LIFT_POWER);
        robot.motorBase.setTargetPosition(robot.LIFT_HIGH_JUNCTION);
    }   // end of liftHighJunction method

    public void liftPosition(int targetPosition){
        robot.motorBase.setPower(robot.LIFT_POWER);
        robot.motorBase.setTargetPosition(targetPosition);
    }   // end of liftPosition method


    /********************************************************************************************
     *  Method: updateError
     *  -   uses the gyro values to determine current angular position.
     *  -   This method will calculate the variance between the current robot angle and the
     *      target angle
     *  -   Note: this method is a sub method of PIDRotate
     * Parameters:
     * @param targetAngle     - angle that the robot would like to turn to
     *******************************************************************************************/
    private double updateError(double targetAngle){
        double calculatedError = 0;

        if (targetAngle > 100 || targetAngle < -100) {
            calculatedError = gyro360(targetAngle) - targetAngle;
        } else {
            calculatedError = getZAngle() - targetAngle;}

        return(calculatedError);
    }

    /*******************************************************************************************
     * Method:      calcDistance
     * Function:    Calculates the distance that the robot has traveled based on a starting set of
     *              encoder values and the current encoder values
     * Parameters:
     * @param heading   - indicates the direction the robot is angled/heading
     * @return
     *******************************************************************************************/
    public double calcDistance(double heading){

        double strafeFactor = 1;        // defaults to 1; changed if the robot is strafing

        if(heading == 90 || heading == -90){
            strafeFactor = robot.STRAFE_FACTOR;
        }

        int totEncoder = Math.abs(robot.motorRF.getCurrentPosition()) + Math.abs(robot.motorLF.getCurrentPosition())
                + Math.abs(robot.motorRR.getCurrentPosition()) + Math.abs(robot.motorLR.getCurrentPosition());

        double avgEncoder = totEncoder/ 4;

        double distanceTraveled = avgEncoder / robot.DRIVE_TICKS_PER_INCH;

        return Math.abs(distanceTraveled * strafeFactor);
    }

    /******************************************************************************************
     * Method:  getZAngle()
     ******************************************************************************************/
    public double getZAngle(){
        return (-robot.imu.getAbsoluteHeading());
    }   // close getZAngle method

    /******************************************************************************************
     * Method:  getZAngleRadians()
     ******************************************************************************************/
    public double getZAngleRadians(){
        return (Math.toRadians(-robot.imu.getAbsoluteHeading()));
    }   // close getZAngle method

    /*******************************************************************************************
     * Method:      updateValues
     * Function:    Prints data to the screen
     * @param action
     * @param initZ
     * @param theta
     * @param currentZ
     * @param zCorrection
     ******************************************************************************************/
    public void updateValues(String action, double initZ, double theta, double currentZ, double zCorrection){
        opMode.telemetry.addData("Current Action = ", action);
        opMode.telemetry.addData("InitZ/targetAngle value  = ", initZ);
        opMode.telemetry.addData("Theta/lastError Value= ", theta);
        opMode.telemetry.addData("CurrentZ/Error Value = ", currentZ);
        opMode.telemetry.addData("zCorrection/derivative Value = ", zCorrection);

        opMode.telemetry.addData("Right Front = ", RF);
        opMode.telemetry.addData("Left Front = ", LF);
        opMode.telemetry.addData("Left Rear = ", LR);
        opMode.telemetry.addData("Right Rear = ", RR);
        opMode.telemetry.update();
    }   // close updateValues method


    /* #########################################################################################
       #########################################################################################
       ################################  CLASS CALCULATIONS ####################################
       #########################################################################################
       #########################################################################################
     */


    /*******************************************************************************************
     * Method gyro360
     *  - Causes the Gyro to behave in 360 mode instead of 180 degree mode
     *******************************************************************************************/
    public double gyro360(double targetAngle){
        double currentZ = getZAngle();
        double rotationalAngle = 0;

        if (targetAngle > 0){
            if ((currentZ >= 0) && (currentZ <= 180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = 180 + (180 + currentZ);
            }// end if(currentZ <=0) - else
        } else {
            if ((currentZ <= 0) && (currentZ >= -180)) {
                rotationalAngle = currentZ;
            } else {
                rotationalAngle = -180 - (180 - currentZ);
            }   // end if(currentZ <=0) - else
        }   // end if(targetAngle >0)-else

        return rotationalAngle;
    }   // end method gyro360

    /*******************************************************************************************
     * Method gyro360
     *  - Causes the Gyro to behave in 360 mode instead of 180 degree mode
     *******************************************************************************************/
    public double gyro360Radians(double targetAngle){
        double rotationalAngle = 0;

        if(targetAngle > Math.PI){
            rotationalAngle = targetAngle - (2 * Math.PI);
        }
        if(targetAngle < -Math.PI){
            rotationalAngle = targetAngle + (2 * Math.PI);
        }

        return rotationalAngle;
    }   // end method gyro360

}   // close the class
