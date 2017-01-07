/*
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: RED Auto Drive By Gyro", group="HardwarePushbot10720")
//@Disabled
public class RED_autonomous_gyro extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot10720         robot   = new HardwarePushbot10720();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 520 ; //520  http://files.andymark.com/am-2964_testing.pdf    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.15;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.15;     // Nominal half speed for better accuracy.
    static final double     DRIVE_SENSOR_SPEED      = 0.015;     // Nominal speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 3 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.05;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.08;     // Larger is more responsive, but also less stable
    String beaconcolor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        robot.sensor.enableLed(false); //set the LED to false

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing
        while (gyro.isCalibrating())  {
            Thread.sleep(50);
            idle();
        }
        telemetry.addData(">", "Robot Ready.");    //

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", gyro.getIntegratedZValue());
            telemetry.update();
            idle();
        }
        gyro.resetZAxisIntegrator();

        telemetry.addData(">", "after the isstarted loop");
        telemetry.update();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn
        // convert the RGB values to HSV values.
/*
        while (opModeIsActive())
        {
            telemetry.addData("heading mode", gyro.getHeadingMode().toString());
            telemetry.addData("integrated z axix value", " == %7d  heading == %7d", gyro.getIntegratedZValue(), gyro.getHeading());
            telemetry.addData("raw values", "x = %7d  y = %7d z=%7d", gyro.rawX(), gyro.rawY(), gyro.rawZ());
            telemetry.update();
        }
*/

        NewgyroDrive(DRIVE_SPEED, 15.0, 0.0);    // Drive FWD 5 inches
        sleep(2000);
        NewgyroTurn(TURN_SPEED, 90);
        sleep(2000);
        NewgyroDrive(DRIVE_SPEED, 15.0, 0.0);    // Drive FWD 25 inches
        sleep(2000);
        NewgyroTurn(TURN_SPEED, -90);
        sleep(2000);
        NewgyroDrive(DRIVE_SPEED, -15.0, 0.0);    // Drive REV 25 inches
        // set the encoder value based on the distance and set the encoders
        // loop till the encoder values are not met
        // while (not reached or stopped)
        //    get z value to see if we are not steering in the right angle
        //    if not then move the robot to make sure that the heading is correct
        //    look at the encoder value to see if the distance is reached
        //    if overshort go back


/*        gyroDrive(DRIVE_SPEED, 5.0, 0.0);    // Drive FWD 48 inches
        telemetry.addData(">", "reached dest");
        telemetry.update();
        Thread.sleep(1000);
        gyroDrive(DRIVE_SPEED, -5.0, 0.0);    // Drive FWD 48 inches
        telemetry.addData(">", "reached dest");
        telemetry.update();

        Thread.sleep(1000);
        gyroDrive(DRIVE_SPEED, 5.0, 0.0);    // Drive FWD 48 inches
        telemetry.addData(">", "reached dest");
        telemetry.update();
  */
    /*
        gyroTurn(TURN_SPEED, 90.0);  // turn right
        telemetry.addData(">", "reached dest");
        telemetry.update();
        gyroDrive(DRIVE_SPEED, 30.0, 0.0);
        //gyroDrive(DRIVE_SPEED, 20.0, 0.0);
        gyroTurn( TURN_SPEED, -90.0);         // Turn  right
        gyroSensorDrive(DRIVE_SENSOR_SPEED ,58.0, 0.0, "blue");
        gyroDrive(DRIVE_SPEED, -40.0, 0.0);
        gyroTurn(TURN_SPEED, 90.0);    // turn right
        gyroDrive(DRIVE_SPEED, 46.0, 0.0);
        telemetry.addData("Path", "Turn Complete");
        telemetry.update();
//        sleep(1000);


        telemetry.addData("Path", "Complete");
        telemetry.update();
*/
    }

    public void NewgyroDrive ( double speed,
                            double distance,
                            double angle) throws InterruptedException {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  zvalue;

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

            telemetry.addData(">>",  "setting new largets Left RIGHT");

            telemetry.addData("Target",  "MoveCount:%7d:  LEFT:%7d:  RIGHT:%7d",    moveCounts,  newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();
            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(-1*newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(-1*speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                zvalue = gyro.getIntegratedZValue();
                if (distance > 0) {
                    if (zvalue < 0) {
                        // steer to left i.e. give more power to the right motor
                        robot.leftMotor.setPower(1.1 * speed);
                        telemetry.addData("Steering Left gave more power to right motor ",zvalue);
                    }
                    if (zvalue > 0) {
                        // steer to left i.e. give more power to the right motor
                        robot.rightMotor.setPower(-1.1 * speed);
                        telemetry.addData("Steering right gave more power to left motor ",zvalue);
                    }
                    if (zvalue == 0) {
                        robot.leftMotor.setPower(speed);
                        robot.rightMotor.setPower(-1 * speed);
                    }
                }
                if (distance < 0) {
                    if (zvalue < 0) {
                        // steer to left i.e. give more power to the right motor
                        robot.rightMotor.setPower(-1.1 * speed);
                    }
                    if (zvalue > 0) {
                        // steer to left i.e. give more power to the right motor
                        robot.leftMotor.setPower(1.1 * speed);
                    }
                    if (zvalue == 0) {
                        robot.leftMotor.setPower(speed);
                        robot.rightMotor.setPower(-1 * speed);
                    }
                }


                // Display drive status for the driver.
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition(), gyro.getIntegratedZValue());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData(">>",  "after move new largets Left RIGHT");
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();

        }
    }

    public void NewgyroTurn ( double speed,
                               double angle) throws InterruptedException {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  zvalue;
        boolean turned;
        double absdiff;

        turned = false;
        gyro.resetZAxisIntegrator(); // zero the angle
        // Ensure that the opmode is still active
        if (opModeIsActive()) {


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (!turned)) {

                zvalue = gyro.getIntegratedZValue();
                absdiff = Math.abs((Math.abs(zvalue) - Math.abs(angle)));
                if (angle > 0) // turn Left i.e. give power to right motor only
                {
                    if (zvalue < angle) {
                        // steer to left i.e. give more power to the right motor
                        robot.rightMotor.setPower(speed);
                        robot.leftMotor.setPower(0);
                        telemetry.addData("not turned enough zvalue less than angle",zvalue);
                        telemetry.addData("abs diff", absdiff);
                        telemetry.update();
                    }
                    if (zvalue > angle) {
                        // steer to left i.e. give more power to the right motor
                        robot.rightMotor.setPower(0);
                        robot.leftMotor.setPower(-1 * speed);
                        telemetry.addData("turned too much zvalue is more than angle", zvalue);
                        telemetry.addData("abs diff", absdiff);
                        telemetry.update();
                    }
                    if (Math.abs(Math.abs(zvalue) - Math.abs(angle)) == 0) {
                        turned = true;
                        robot.leftMotor.setPower(0);
                        robot.rightMotor.setPower(0);
                        telemetry.addData("Reached angle ", zvalue);
                        telemetry.addData("abs diff", absdiff);
                        telemetry.update();
                    }
                }
                if (angle < 0) // turn right i.e. give power to left motor only
                {
                    if (zvalue > angle) {
                        // steer to left i.e. give more power to the right motor
                        robot.rightMotor.setPower(0);
                        robot.leftMotor.setPower(-1 * speed);
                        telemetry.addData("not turned enough zvalue less than angle",zvalue);
                        telemetry.addData("abs diff", absdiff);
                        telemetry.update();
                    }
                    if (zvalue < angle) {
                        // steer to left i.e. give more power to the right motor
                        robot.leftMotor.setPower(0);
                        robot.rightMotor.setPower(1 * speed);
                        telemetry.addData("not turned enough zvalue less than angle",zvalue);
                        telemetry.addData("abs diff", absdiff);
                        telemetry.update();
                    }
                    if (Math.abs(Math.abs(zvalue) - Math.abs(angle)) == 0) {
                        turned = true;
                        robot.leftMotor.setPower(0);
                        robot.rightMotor.setPower(0);
                        telemetry.addData("Reached angle ", zvalue);
                        telemetry.addData("abs diff", absdiff);
                        telemetry.update();
                    }
                }



                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

        }
    }

    public String getcolor(int blue, int red, int green)
    {
        telemetry.addData(">", "red color %d", red);
        telemetry.addData(">", "blue color %d", blue);
        telemetry.addData(">", "green color %d", green);
        telemetry.update();
        if (blue > red && blue > green && blue >=3) return "Blue";
        if (red > blue && red > green && red >= 3)  return "Red";
        else return "Unknown";
    }

    public void NewgyroSensorDrive ( double speed,
                               double distance,
                               double angle) throws InterruptedException {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  zvalue;
        boolean ispressed;

        ispressed = false;

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

            telemetry.addData(">>",  "setting new largets Left RIGHT");

            telemetry.addData("Target",  "MoveCount:%7d:  LEFT:%7d:  RIGHT:%7d",    moveCounts,  newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();
            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(-1*newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(-1*speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                for (int i = 0; i< 2; i++) {

                    beaconcolor = getcolor(robot.sensor.blue(), robot.sensor.red(), robot.sensor.green());
                    //  sleep(1000);
                    telemetry.addData(">", "read color %s", beaconcolor);
                    telemetry.update();
                }
                if ((ispressed == false) && ( beaconcolor == "Red")) {
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                    telemetry.addData("It Sensed Color ", "Now pushing the beacon button" );
                    telemetry.update();
                    robot.sensorMotor.setPower(-0.1);
                    sleep(2400);
                    robot.sensorMotor.setPower(0.1);
                    sleep(2400);
                    robot.sensorMotor.setPower(0);
                    ispressed = true;
                }
                telemetry.addData("End of Color Sensor Code", "Done Color Detection" );
                telemetry.update();

                zvalue = gyro.getIntegratedZValue();
                if (distance > 0) {
                    if (zvalue < 0) {
                        // steer to left i.e. give more power to the right motor
                        robot.leftMotor.setPower(1.1 * speed);
                        telemetry.addData("Steering Left gave more power to right motor ",zvalue);
                    }
                    if (zvalue > 0) {
                        // steer to left i.e. give more power to the right motor
                        robot.rightMotor.setPower(-1.1 * speed);
                        telemetry.addData("Steering right gave more power to left motor ",zvalue);
                    }
                    if (zvalue == 0) {
                        robot.leftMotor.setPower(speed);
                        robot.rightMotor.setPower(-1 * speed);
                    }
                }
                if (distance < 0) {
                    if (zvalue < 0) {
                        // steer to left i.e. give more power to the right motor
                        robot.rightMotor.setPower(-1.1 * speed);
                    }
                    if (zvalue > 0) {
                        // steer to left i.e. give more power to the right motor
                        robot.leftMotor.setPower(1.1 * speed);
                    }
                    if (zvalue == 0) {
                        robot.leftMotor.setPower(speed);
                        robot.rightMotor.setPower(-1 * speed);
                    }
                }


                // Display drive status for the driver.
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition(), gyro.getIntegratedZValue());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData(">>",  "after move new largets Left RIGHT");
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();

        }
    }


    public void gyroSensorDrive ( double speed,
                            double distance,
                            double angle, String color) throws InterruptedException {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        boolean ispressed;

        ispressed = false;

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

            telemetry.addData(">>",  "setting new largets Left RIGHT");

            telemetry.addData("Target",  "MoveCount:%7d:  LEFT:%7d:  RIGHT:%7d",    moveCounts,  newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();
            sleep(1000);
            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(-1*newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(-1*speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                for (int i = 0; i< 2; i++) {

                    beaconcolor = getcolor(robot.sensor.blue(), robot.sensor.red(), robot.sensor.green());
                  //  sleep(1000);
                    telemetry.addData(">", "read color %s", beaconcolor);
                    telemetry.update();
                }
                if ((ispressed == false) && ( beaconcolor == "Red")) {
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                    telemetry.addData("It Sensed Color ", "Now pushing the beacon button" );
                    telemetry.update();
                    robot.sensorMotor.setPower(-0.1);
                    sleep(2400);
                    robot.sensorMotor.setPower(0.1);
                    sleep(2400);
                    robot.sensorMotor.setPower(0);
                    ispressed = true;
                }
                telemetry.addData("End of Color Sensor Code", "Done Color Detection" );
                telemetry.update();

                robot.leftMotor.setPower(leftSpeed);
                robot.rightMotor.setPower(-1*rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData(">>",  "after move new largets Left RIGHT");
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();
            //  sleep(5000);

        }
    }


    /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) throws InterruptedException {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightMotor.getCurrentPosition() + moveCounts;

            telemetry.addData(">>",  "setting new largets Left RIGHT");

            telemetry.addData("Target",  "MoveCount:%7d:  LEFT:%7d:  RIGHT:%7d",    moveCounts,  newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();
            // Set Target and Turn On RUN_TO_POSITION
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(-1*newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(-1*speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftMotor.setPower(leftSpeed);
                robot.rightMotor.setPower(-1*rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                                                             robot.rightMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                sleep(2000);
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            telemetry.addData(">>",  "after move new largets Left RIGHT");
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
            telemetry.addData("Actual",  "%7d:%7d",      robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();
          //  sleep(5000);

        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @throws InterruptedException
     */
    public void gyroTurn (  double speed, double angle)
                              throws InterruptedException {

        // keep looping while we are still active, and not on heading.
        gyro.resetZAxisIntegrator();
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            idle();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     * @throws InterruptedException
     */
    public void gyroHold( double speed, double angle, double holdTime)
                            throws InterruptedException {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        gyro.resetZAxisIntegrator();
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
            idle();
        }

        // Stop all motion;
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        double heading = gyro.getHeading();

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error)<=HEADING_THRESHOLD )
        {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
            telemetry.addData(">", "REACHED ON TARGET");
            telemetry.update();
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftMotor.setPower(leftSpeed);
        robot.rightMotor.setPower(-1*rightSpeed);

        // Display it for the driver.
        telemetry.addData("Head/Trgt", "%5.2f/%5.2f", heading,angle);
        telemetry.addData("Err/Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        telemetry.update();

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {
        double heading = gyro.getHeading(); // heading is returned 0 to 360, positive turns clockwise
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = heading - targetAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
