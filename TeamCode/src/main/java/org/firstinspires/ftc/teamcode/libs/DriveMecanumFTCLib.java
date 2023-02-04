package org.firstinspires.ftc.teamcode.libs;

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
        int lfStart = robot.motorLF.getCurrentPosition();
        int lrStart = robot.motorLR.getCurrentPosition();
        int rfStart = robot.motorRF.getCurrentPosition();
        int rrStart = robot.motorRR.getCurrentPosition();

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

            distanceTraveled = calcDistance(heading, rfStart, rrStart, lfStart, lrStart);
            if(distanceTraveled >= distance) active = false;
            opMode.telemetry.addData("Distance Traveled = ", distanceTraveled);
            opMode.telemetry.addData("Distance = ", distance);
            opMode.telemetry.update();
            opMode.idle();

        }   // end of while loop
        motorsHalt();
    }   // close driveDistance method


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















    /********************************************************************************************
     * Method:      PIDRotate
     * Function:    Rotates the robot to the desired angle using a PID algorithm controled by the
     *              gyro sensor
     * Parameters:
     * @param targetAngle -> desire ending angle/position of the robot
     * @param targetError -> how close should the robot get to the desired angle
     *******************************************************************************************/
    public void oldPIDRotate(double targetAngle, double targetError){
        double integral = 0;
        double error;
        double Cp = 0.06;
        double Ci = 0.0003;
        double Cd = 0.0001;
        double maxSpeed = 0.5;
        double rotationSpeed;
        double derivative = 0, deltaError, lastError=0;

        // check to see how far the robot is rotating to decide which gyro sensor value to use
        error = updateError(targetAngle);

        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
            while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
                deltaError = lastError - error;
                rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative)) * maxSpeed;

                // Clip motor speed
                rotationSpeed = Range.clip(rotationSpeed, -maxSpeed, maxSpeed);

                // bump the power up to min threshold if the robot doesn't have enough power to turn
                if ((rotationSpeed > -0.15) && (rotationSpeed < 0)) {
                    rotationSpeed = -robot.MIN_PIDROTATE_POWER;
                } else if ((rotationSpeed < 0.15) && (rotationSpeed > 0)) {
                    rotationSpeed = robot.MIN_PIDROTATE_POWER;
                }

                RF = rotationSpeed;
                LF = -rotationSpeed;
                LR = -rotationSpeed;
                RR = rotationSpeed;

                setDrivePower(RF, LF, LR, RR);

                lastError = error;

                // check to see how far the robot is rotating to decide which gyro sensor value to use
                error = updateError(targetAngle);

            }   // end of while Math.abs(error)
            motorsHalt();

            // Perform a final calc on the error to confirm that the robot didn't overshoot the
            // target position after the last measurement was taken.
            opMode.sleep(10);
            error = updateError(targetAngle);
        }   // close outside while loop

        // shut off the drive motors
        motorsHalt();

    }   //end of the PIDRotate Method


    /********************************************************************************************
     *  Method: driveByTime
     *  -   uses time (based on the clock value) to drive to a heading
     *  -   This method will autocorrect the position of the robot if it drifts off its position
     *  -   Note: This method uses gyro360 to measure it's angle. The reference target angle used
     *              is 0 degrees as that ensures that the reference and measured angles always
     *              provide consistent reporting comparisons.
     * Parameters:
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param duration  - amount of time that the robot will move
     ********************************************************************************************/
    public void driveByTime(double power, double heading, double duration) {
        String action = "Initializing";
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double theta = Math.toRadians(90 + heading);
        ElapsedTime runTime = new ElapsedTime();

        if(runTime.time() >= duration) active = false;

        while(opMode.opModeIsActive() && active) {
            updateValues(action, initZ, theta, currentZ, zCorrection);

            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if(runTime.time() >= duration) active = false;

            // check to see if the inital gyro value is less than 90 degrees
            // if so, use the gyro360 value to determine drift
            if (initZ >90 || initZ < -90){
                currentZ = getZAngle();
//                currentZ = -gyro360(0);      // always use 0 as the reference angle
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
                    action = " initZ < currentZ";
                }
                if (initZ > currentZ) {
                    RF = RF - zCorrection;
                    RR = RR - zCorrection;
                    LF = LF + zCorrection;
                    LR = LR + zCorrection;
                    action = " initZ < currentZ";
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

        }   // end of while loop

        motorsHalt();
    }   // close driveByTime method



    /********************************************************************************************
     *  Method: driveSensorDistance
     *  -   uses time (based on the clock value) to drive to a heading
     *  -   This method will autocorrect the position of the robot if it drifts off its position
     *  -   Note: This method uses gyro360 to measure it's angle. The reference target angle used
     *              is 0 degrees as that ensures that the reference and measured angles always
     *              provide consistent reporting comparisons.
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param stopDistance  - distance robot will be from object when stop request
     */
    public void driveSensorDistance(double power, double heading, double stopDistance) {
        String action = "Initializing";
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double theta = Math.toRadians(90 + heading);

        if(robot.sensorWall.getDistance(DistanceUnit.INCH) < stopDistance) active = false;

        while(opMode.opModeIsActive() && active) {
            updateValues(action, initZ, theta, currentZ, zCorrection);

            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if(robot.sensorWall.getDistance(DistanceUnit.INCH) < stopDistance) active = false;

            // check to see if the inital gyro value is less than 90 degrees
            // if so, use the gyro360 value to determine drift
            if (initZ >90 || initZ < -90){
                currentZ = getZAngle();
//                currentZ = -gyro360(0);      // always use 0 as the reference angle
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
                    action = " initZ < currentZ";
                }
                if (initZ > currentZ) {
                    RF = RF - zCorrection;
                    RR = RR - zCorrection;
                    LF = LF + zCorrection;
                    LR = LR + zCorrection;
                    action = " initZ < currentZ";
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

        }   // end of while loop

        motorsHalt();
    }   // close driveSensorDistance method


    /******************************************************************************************
     * Method:      driveSensorDistanceOut
     * Function:    Drives away from an object until it reaches the target distance as measured
     *              by the sensor
     * @param power     - Speed at which to drive
     * @param heading   - Direction to drive
     * @param stopDistance  - distance from starting object to stop
     ******************************************************************************************/
    public void driveSensorDistanceOut(double power, double heading, double stopDistance) {
        String action = "Initializing";
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double theta = Math.toRadians(90 + heading);

        if(robot.sensorWall.getDistance(DistanceUnit.INCH) > stopDistance) active = false;

        while(opMode.opModeIsActive() && active) {
            updateValues(action, initZ, theta, currentZ, zCorrection);

            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if(robot.sensorWall.getDistance(DistanceUnit.INCH) > stopDistance) active = false;

            // check to see if the inital gyro value is less than 90 degrees
            // if so, use the gyro360 value to determine drift
            if (initZ >90 || initZ < -90){
                currentZ = getZAngle();
//                currentZ = -gyro360(0);      // always use 0 as the reference angle
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
                    action = " initZ < currentZ";
                }
                if (initZ > currentZ) {
                    RF = RF - zCorrection;
                    RR = RR - zCorrection;
                    LF = LF + zCorrection;
                    LR = LR + zCorrection;
                    action = " initZ < currentZ";
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

        }   // end of while loop

        motorsHalt();
    }   // close driveSensorDistanceOut method


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
        robot.motorRF.set(0);
        robot.motorLF.set(0);
        robot.motorLR.set(0);
        robot.motorRR.set(0);
    }   // end of motorsHalt method


    /* #########################################################################################
       #########################################################################################
       ################################  MECHANISM METHODS #####################################
       #########################################################################################
       #########################################################################################
     */

    public void closeClaw(){
        robot.servoGrabber.setPosition(robot.SERVO_GRAB_CLOSE);
    }   // end of closeClaw method

    public void openClaw(){
        robot.servoGrabber.setPosition(robot.SERVO_GRAB_OPEN);
    }   // end of openClaw method

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
     * @param rfStart
     * @param rrStart
     * @param lfStart
     * @param lrStart
     * @return
     *******************************************************************************************/
    public double calcDistance(double heading, int rfStart, int rrStart, int lfStart, int lrStart){

        double strafeFactor = 1;        // defaults to 1; changed if the robot is strafing
        int rfEncoder = robot.motorRF.getCurrentPosition();
        int lfEncoder = robot.motorLF.getCurrentPosition();
        int rrEncoder = robot.motorRR.getCurrentPosition();
        int lrEncoder = robot.motorLR.getCurrentPosition();

        if(heading == 90 || heading == -90){
            strafeFactor = robot.STRAFE_FACTOR;
        }

        int totEncoder = Math.abs(rfStart - rfEncoder) + Math.abs(lfStart - lfEncoder)
                + Math.abs(rrStart-rrEncoder) + Math.abs(lrStart - lrEncoder);

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
    /*
     * Method calcRPM()
     */
    public double calcRPM(double tick0, double time0, double tick1, double time1){
        double rPM = (((Math.abs(tick0-tick1)/robot.GB_COUNTS_PER_ROTATION))/(Math.abs(time1-time0)))*60;
        return (rPM);
    }   // close calcRPM method

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

}   // close the class
