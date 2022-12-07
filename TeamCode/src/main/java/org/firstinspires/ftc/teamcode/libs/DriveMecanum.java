package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.HardwareProfile;

public class DriveMecanum {

    private HardwareProfile robot;
    public double RF, LF, LR, RR;
    public LinearOpMode opMode;

    /*
     * Constructor method
     */
    public DriveMecanum(HardwareProfile myRobot, LinearOpMode myOpMode){
        robot = myRobot;
        opMode = myOpMode;

    }   // close DriveMecanum constructor Method


    public void driveDistance(double power, double heading, double distance) {
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double distanceTraveled = 0;

        double theta = Math.toRadians(90 + heading);
        double lfStart = robot.motorLF.getCurrentPosition();
        double lrStart = robot.motorLR.getCurrentPosition();
        double rfStart = robot.motorRF.getCurrentPosition();
        double rrStart = robot.motorRR.getCurrentPosition();

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
                    RF = RF - zCorrection;
                    RR = RR - zCorrection;
                    LF = LF + zCorrection;
                    LR = LR + zCorrection;
                }
                if (initZ > currentZ) {
                    RF = RF + zCorrection;
                    RR = RR + zCorrection;
                    LF = LF - zCorrection;
                    LR = LR - zCorrection;
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
        double integral = 0;
        int iterations = 0;
        ElapsedTime timeElapsed = new ElapsedTime();
        double startTime = timeElapsed.time();
        double totalTime;
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

                RF = -rotationSpeed;
                LF = rotationSpeed;
                LR = rotationSpeed;
                RR = -rotationSpeed;

                setDrivePower(RF, LF, LR, RR);

                lastError = error;
                iterations++;

                /*
                opMode.telemetry.addData("InitZ/targetAngle value  = ", targetAngle);
                opMode.telemetry.addData("Current Angle  = ", getZAngle());
                opMode.telemetry.addData("Theta/lastError Value= ", lastError);
                opMode.telemetry.addData("CurrentZ/Error Value = ", error);
                opMode.telemetry.addData("zCorrection/derivative Value = ", derivative);

                opMode.telemetry.addData("Right Front = ", RF);
                opMode.telemetry.addData("Left Front = ", LF);
                opMode.telemetry.addData("Left Rear = ", LR);
                opMode.telemetry.addData("Right Rear = ", RR);
                opMode.telemetry.update();
                 */

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

        /*
        totalTime = timeElapsed.time() - startTime;
        opMode.telemetry.addData("Iterations = ", iterations);
        opMode.telemetry.addData("Final Angle = ", getZAngle());
        opMode.telemetry.addData("Total Time Elapsed = ", totalTime);
        opMode.telemetry.update();
         */
    }   //end of the PIDRotate Method


    /**
     *  Method: updateError
     *  -   uses the gyro values to determine current angular position.
     *  -   This method will calculate the variance between the current robot angle and the target angle
     *  -   Note: this method is a sub method of PIDRotate
     * @param targetAngle     - angle that the robot would like to turn to
     */
    private double updateError(double targetAngle){
        double calculatedError = 0;

        if (targetAngle > 100 || targetAngle < -100) {
            calculatedError = gyro360(targetAngle) - targetAngle;
        } else {
            calculatedError = getZAngle() - targetAngle;}

        return(calculatedError);
    }




    /*
     *  Method: robotCorrect
     *  -   uses time (based on the clock value) to drive to a heading
     *  -   This method will autocorrect the position of the robot if it drifts off its position
     *  -   Note: This method uses gyro360 to measure it's angle. The reference target angle used
     *              is 0 degrees as that ensures that the reference and measured angles always
     *              provide consistent reporting comparisons.
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param duration  - amount of time that the robot will move
     */
    public void robotCorrect(double power, double heading, double duration) {
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
            if(RF > 1) RF = 1;
            else if (RF < -1) RF = -1;

            if(LF > 1) LF = 1;
            else if (LF < -1) LF = -1;

            if(LR > 1) LR = 1;
            else if (LR < -1) LR = -1;

            if(RR > 1) RR = 1;
            else if (RR < -1) RR = -1;

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);

        }   // end of while loop

        motorsHalt();
    }   // close robotCorrect method






    /*
     *  Method: robotCorrect
     *  -   uses time (based on the clock value) to drive to a heading
     *  -   This method will autocorrect the position of the robot if it drifts off its position
     *  -   Note: This method uses gyro360 to measure it's angle. The reference target angle used
     *              is 0 degrees as that ensures that the reference and measured angles always
     *              provide consistent reporting comparisons.
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param duration  - amount of time that the robot will move
     */
    public void driveSensorDistance(double power, double heading, double stopDistance) {
        String action = "Initializing";
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double theta = Math.toRadians(90 + heading);
        ElapsedTime runTime = new ElapsedTime();

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
            if(RF > 1) RF = 1;
            else if (RF < -1) RF = -1;

            if(LF > 1) LF = 1;
            else if (LF < -1) LF = -1;

            if(LR > 1) LR = 1;
            else if (LR < -1) LR = -1;

            if(RR > 1) RR = 1;
            else if (RR < -1) RR = -1;

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);

            opMode.telemetry.addData("LF Current = ", robot.motorLF.getCurrentPosition());
            opMode.telemetry.addData("RF Current = ", robot.motorRF.getCurrentPosition());
            opMode.telemetry.addData("LR Current = ", robot.motorLR.getCurrentPosition());
            opMode.telemetry.addData("RR Current = ", robot.motorRR.getCurrentPosition());
            opMode.telemetry.addData( "element Distance = ",  robot.sensorWall.getDistance(DistanceUnit.INCH));
            opMode.telemetry.update();

        }   // end of while loop

        motorsHalt();
    }   // close robotCorrect method
    public void driveSensorDistanceOut(double power, double heading, double stopDistance) {
        String action = "Initializing";
        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;
        double theta = Math.toRadians(90 + heading);
        ElapsedTime runTime = new ElapsedTime();

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
            if(RF > 1) RF = 1;
            else if (RF < -1) RF = -1;

            if(LF > 1) LF = 1;
            else if (LF < -1) LF = -1;

            if(LR > 1) LR = 1;
            else if (LR < -1) LR = -1;

            if(RR > 1) RR = 1;
            else if (RR < -1) RR = -1;

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);

            opMode.telemetry.addData("LF Current = ", robot.motorLF.getCurrentPosition());
            opMode.telemetry.addData("RF Current = ", robot.motorRF.getCurrentPosition());
            opMode.telemetry.addData("LR Current = ", robot.motorLR.getCurrentPosition());
            opMode.telemetry.addData("RR Current = ", robot.motorRR.getCurrentPosition());
            opMode.telemetry.addData( "element Distance = ",  robot.sensorWall.getDistance(DistanceUnit.INCH));
            opMode.telemetry.update();

        }   // end of while loop

        motorsHalt();
    }   // close robotCorrect method

    /*
     *  Method: robotNoCorrect
     *  -   uses time (based on the clock value) to drive to a heading
     *  -   This method will autocorrect the position of the robot if it drifts off its position
     *  -   Note: This method uses gyro360 to measure it's angle. The reference target angle used
     *              is 0 degrees as that ensures that the reference and measured angles always
     *              provide consistent reporting comparisons.
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param duration  - amount of time that the robot will move
     */
    public void robotNoCorrect(double power, double heading, double duration) {
        boolean active = true;
        double theta = Math.toRadians(90 + heading);
        ElapsedTime runTime = new ElapsedTime();

        if(runTime.time() >= duration) active = false;

        while(opMode.opModeIsActive() && active) {

            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if(runTime.time() >= duration) active = false;

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);

        }   // end of while loop

        motorsHalt();
    }   // close robotCorrect method

    /**
     * Method shooterControl()
     * @param targetRPM
     */
   // public void shooterControl(double targetRPM){
     //   robot.motorTurnTable.setVelocity(rpmToTicksPerSecond(targetRPM));
  //  }   // end of method shooterControl

    /**
     * method rpmToTicksPerSecond
     * @param targetRPM
     */
    private double rpmToTicksPerSecond(double targetRPM){
        return (targetRPM * 28 / 60);
    }   // end of method rpmToTicksPerSecond

    /*
     *  Method: driveDistance
     *  -   uses the encoder values to determine distance traveled.
     *  -   This method will autocorrect the position of the robot if it drifts off its position
     *  -   Note: This method uses gyro360 to measure it's angle. The reference target angle used
     *              is 0 degrees as that ensures that the reference and measured angles always
     *              provide consistent reporting comparisons.
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param distance  - amount of time that the robot will move
     */
    public void driveDistance_old(double power, double heading, double distance) {
        String action = "Initializing";
        /*
        robot.motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         */

        double initZ = getZAngle();
        double currentZ = 0;
        double zCorrection = 0;
        boolean active = true;

        double theta = Math.toRadians(90 + heading);
        double lfStart = robot.motorLF.getCurrentPosition();
        double lrStart = robot.motorLR.getCurrentPosition();
        double rfStart = robot.motorRF.getCurrentPosition();
        double rrStart = robot.motorRR.getCurrentPosition();
        double elementDistance;
        /*
        opMode.telemetry.addData("LF Start = ", lfStart);
        opMode.telemetry.addData("RF Start = ", rfStart);
        opMode.telemetry.addData("LR Start = ", lrStart);
        opMode.telemetry.addData("RR Start = ", rrStart);
        opMode.telemetry.addData("Distance = ", distance);
        opMode.telemetry.addData("Heading = ", heading);
        opMode.telemetry.addData("Calculated Distance = ", calcDistance(heading, lfStart, lrStart, rfStart, rrStart));
        opMode.telemetry.update();
        opMode.sleep (5000);

         */

        while(opMode.opModeIsActive() && active) {
            updateValues(action, initZ, theta, currentZ, zCorrection);

            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            if (initZ >90 || initZ < -90){
                currentZ = -gyro360(0);      // always use 0 as the reference angle
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
            if(RF > 1) RF = 1;
            else if (RF < -1) RF = -1;

            if(LF > 1) LF = 1;
            else if (LF < -1) LF = -1;

            if(LR > 1) LR = 1;
            else if (LR < -1) LR = -1;

            if(RR > 1) RR = 1;
            else if (RR < -1) RR = -1;

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);
            opMode.telemetry.addData("LF Start = ", lfStart);
            opMode.telemetry.addData("Distance = ", distance);
            opMode.telemetry.addData("Heading = ", heading);
            opMode.telemetry.addData("Calculated Distance = ", calcDistance(heading, rfStart, rrStart, lfStart, lrStart));
            opMode.telemetry.update();
            if(calcDistance(heading, rfStart, rrStart, lfStart, lrStart) >= distance) active = false;
            opMode.idle();

        }   // end of while loop

        motorsHalt();

        opMode.telemetry.addData("LF Start = ", lfStart);
        opMode.telemetry.addData("RF Start = ", rfStart);
        opMode.telemetry.addData("LR Start = ", lrStart);
        opMode.telemetry.addData("RR Start = ", rrStart);
        opMode.telemetry.addData("Distance = ", distance);
        opMode.telemetry.addData("Heading = ", heading);
        opMode.telemetry.addData("Calculated Distance = ", calcDistance(heading, rfStart, rrStart, lfStart, lrStart));
        opMode.telemetry.addData("LF Current = ", robot.motorLF.getCurrentPosition());
        opMode.telemetry.addData("RF Current = ", robot.motorRF.getCurrentPosition());
        opMode.telemetry.addData("LR Current = ", robot.motorLR.getCurrentPosition());
        opMode.telemetry.addData("RR Current = ", robot.motorRR.getCurrentPosition());
        opMode.telemetry.addData( "element Distance = ",  robot.sensorWall.getDistance(DistanceUnit.INCH));
        opMode.telemetry.update();
//        opMode.sleep (20000);


    }   // close driveDistnace method

    /**
     *  Method: driveSimpleDistance
     *  -   uses the encoder values to determine distance traveled.
     *  -   This method will autocorrect the position of the robot if it drifts off its position
     *  -   Note: This method uses gyro360 to measure it's angle. The reference target angle used
     *              is 0 degrees as that ensures that the reference and measured angles always
     *              provide consistent reporting comparisons.
     * @param power     - provides the power/speed that the robot should move
     * @param heading   - direction for the robot to strafe to
     * @param distance  - amount of time that the robot will move
     */
    public void driveSimpleDistance(double power, double heading, double distance) {
        boolean active = true;

        double theta = Math.toRadians(90 + heading);
        double lfStart = robot.motorLF.getCurrentPosition();
        double lrStart = robot.motorLR.getCurrentPosition();
        double rfStart = robot.motorRF.getCurrentPosition();
        double rrStart = robot.motorRR.getCurrentPosition();

        while(opMode.opModeIsActive() && active) {

            RF = power * (Math.sin(theta) + Math.cos(theta));
            LF = power * (Math.sin(theta) - Math.cos(theta));
            LR = power * (Math.sin(theta) + Math.cos(theta));
            RR = power * (Math.sin(theta) - Math.cos(theta));

            /*
             * Apply power to the drive wheels
             */
            setDrivePower(RF, LF, LR, RR);

            if(calcDistance(heading, rfStart, rrStart, lfStart, lrStart) >= distance) active = false;

        }   // end of while loop

        motorsHalt();
    }   // close driveDistnace method

    /*
     *  Method: calcDistance
     *  @param heading  - indicates the direction the robot is angled/heading
     */
    public double calcDistance(double heading, double rfStart, double rrStart, double lfStart, double lrStart){

        double strafeFactor = 1;
        double distanceTraveled = 0;
        double rfEncoder = robot.motorRF.getCurrentPosition();
        double lfEncoder = robot.motorLF.getCurrentPosition();
        double rrEncoder = robot.motorRR.getCurrentPosition();
        double lrEncoder = robot.motorLR.getCurrentPosition();

        if(heading == 90 || heading == -90){
            strafeFactor = robot.STRAFE_FACTOR;
        }

            distanceTraveled = ((Math.abs(rfStart - rfEncoder) + Math.abs(lfStart - lfEncoder)
                    + Math.abs(rrStart-rrEncoder) + Math.abs(lrStart - lrEncoder))/4) / robot.DRIVE_TICKS_PER_INCH;

        return Math.abs(distanceTraveled * strafeFactor);
    }


    /*
     * Method getZAngle()
     */
    public double getZAngle(){
        return (-robot.imu.getAngularOrientation().firstAngle);
    }   // close getZAngle method
    /*
     * Method calcRPM()
     */
    public double calcRPM(double tick0, double time0, double tick1, double time1){
        double rPM = (((Math.abs(tick0-tick1)/robot.GB_COUNTS_PER_ROTATION))/(Math.abs(time1-time0)))*60;
        return (rPM);
    }   // close calcRPM method

    /*
     * Method shooterPower()
     */
    public double shooterPower(double shooterPower, double currentRPM, double elapsedTime, double targetRPM){

        double error = (targetRPM - currentRPM)/1000;
        double integral = error * elapsedTime;
//        double integral = 1;
        double Cp = 0.015; //0.003
        double Ci = 0.0003;
        double Cd = 0.0001;
        double maxPower = 0.7;
        double derivative = 0, deltaError, lastError=0;

        double powerAdjust = ((Cp*error) + (Ci * integral) + (Cd * derivative)) * maxPower;
        opMode.telemetry.addData("powerAdjust = ", powerAdjust);
        shooterPower = shooterPower + powerAdjust;
        return (Range.clip(shooterPower,0, maxPower));
//        return (0.70);
    }   // close shooterPower method

    /*
     * Method Clipspeed
     */
    public double clipSpeed (double motorSpeed){
        if(motorSpeed > 1) return 1;
        else if(motorSpeed < -1) return -1;
        else return motorSpeed;
    }   // end of method clipSpeed

    /*
     * Method updateValues
     */
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

    public void closeClaw(){
        robot.servoGrabber.setPosition(robot.SERVO_GRAB_CLOSE);
    }
    public void openClaw(){
        robot.servoGrabber.setPosition(robot.SERVO_GRAB_OPEN);
    }


    /**
     * Method: PIDRotate
     * Parameters:
     * @param targetAngle -> desire ending angle/position of the robot
     * @param targetError -> how close should the robot get to the desired angle
     */
    public void PIDRotate_OLD(double targetAngle, double targetError){
        double integral = 0;
        int iterations = 0;
        ElapsedTime timeElapsed = new ElapsedTime();
        double startTime = timeElapsed.time();
        double totalTime;
        double error;
        double Cp = 0.06;
        double Ci = 0.0003;
        double Cd = 0.0001;
        double maxSpeed = 0.5;
        double rotationSpeed;
        double derivative = 0, deltaError, lastError=0;

        // check to see how far the robot is rotating to decide which gyro sensor value to use
        if(targetAngle > 90 || targetAngle < -90){
            error = gyro360(targetAngle) - targetAngle;
        } else {
            error = getZAngle() - targetAngle;
        }

        // nested while loops are used to allow for a final check of an overshoot situation
        while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
            while ((Math.abs(error) >= targetError) && opMode.opModeIsActive()) {
                deltaError = lastError - error;
                rotationSpeed = ((Cp * error) + (Ci * integral) + (Cd * derivative)) * maxSpeed;

                // Clip motor speed
                rotationSpeed = Range.clip(rotationSpeed, -maxSpeed, maxSpeed);

                if ((rotationSpeed > -0.25) && (rotationSpeed < 0)) {
                    rotationSpeed = -0.19;
                } else if ((rotationSpeed < 0.25) && (rotationSpeed > 0)) {
                    rotationSpeed = 0.19;
                }

                RF = rotationSpeed;
                LF = -rotationSpeed;
                LR = -rotationSpeed;
                RR = rotationSpeed;

                setDrivePower(RF, LF, LR, RR);

                lastError = error;
                iterations++;

                opMode.telemetry.addData("InitZ/targetAngle value  = ", targetAngle);
                opMode.telemetry.addData("Current Angle  = ", getZAngle());
                opMode.telemetry.addData("Theta/lastError Value= ", lastError);
                opMode.telemetry.addData("CurrentZ/Error Value = ", error);
                opMode.telemetry.addData("zCorrection/derivative Value = ", derivative);

                opMode.telemetry.addData("Right Front = ", RF);
                opMode.telemetry.addData("Left Front = ", LF);
                opMode.telemetry.addData("Left Rear = ", LR);
                opMode.telemetry.addData("Right Rear = ", RR);
                opMode.telemetry.update();

                // check to see how far the robot is rotating to decide which gyro sensor value to use
                if (targetAngle > 90 || targetAngle < -90) {
                    error = gyro360(targetAngle) - targetAngle;
                } else {
                    error = getZAngle() - targetAngle;
                }

            }   // end of while Math.abs(error)
            motorsHalt();

            // Perform a final calc on the error to confirm that the robot didn't overshoot the
            // target position after the last measurement was taken.
//            opMode.sleep(5);
            if (targetAngle > 90 || targetAngle < -90) {
                error = gyro360(targetAngle) - targetAngle;
            } else {
                error = getZAngle() - targetAngle;
            }
        }

        // shut off the drive motors
        motorsHalt();

        totalTime = timeElapsed.time() - startTime;
        opMode.telemetry.addData("Iterations = ", iterations);
        opMode.telemetry.addData("Final Angle = ", getZAngle());
        opMode.telemetry.addData("Total Time Elapsed = ", totalTime);
        opMode.telemetry.update();
    }   //end of the PIDRotate Method

    /*
     * Method gyro360
     *  - Causes the Gyro to behave in 360 mode instead of 180 degree mode
     */
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

    private double convertToInches(double convValue){

        return (convValue * robot.DRIVE_TICKS_PER_INCH);

    }   // end of returnInches method

    /**
     * Sets power to all four drive motors
     * @param RF power for right front motor
     * @param LF power for left front motor
     * @param LR power for left rear motor
     * @param RR power for right rear motor
     */
    public void setDrivePower(double RF, double LF, double LR, double RR){
        robot.motorRF.setPower(RF);
        robot.motorLF.setPower(LF);
        robot.motorLR.setPower(LR);
        robot.motorRR.setPower(RR);
    }

    public void shootRings(){
        // fire the rings
        //robot.servoRingStopper.setPosition(robot.SERVO_SHOOTER_UP);
        opMode.sleep(200);
        //robot.servoRingStopper.setPosition(robot.SERVO_SHOOTER_DOWN);
        // Decide the next state based on the number of rings on the floor


        //robot.motorIntake.setPower(0);
    }
    /*
     * Method motorsHalt
     */
    public void motorsHalt(){
        robot.motorRF.setPower(0);
        robot.motorLF.setPower(0);
        robot.motorLR.setPower(0);
        robot.motorRR.setPower(0);
    }   // end of motorsHalt method

    /**
     * Sets power to all four drive motors
     * @param globalPositionUpdate class for collecting global positioning information
     * @param targetXPosition provides the target X coordinate
     * @param targetYPosition provides the target Y coordinate
     * @param power speed of the robot
     * @param robotOrientation indicates what the orientation of the robot should be at the target coordinates
     * @param distanceError provides details for how close to get to the target coordinates
     */

}   // close the class
