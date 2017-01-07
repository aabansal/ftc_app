package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by bansa on 12/30/2016.
 */
//@Disabled
public class robotMovements{//} extends LinearOpMode{
    HardwarePushbot10720         robot  = new HardwarePushbot10720();   // Use a Pushbot's hardware//

    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device

    static final double     COUNTS_PER_MOTOR_REV    = 295 ;    // eg: TETRIX Motor Encoder
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

    public void NewgyroDrive ( double speed,
                                   double distance,
                                   double angle, Telemetry telemetry) throws InterruptedException {

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
        //if (opModeIsActive())
        {

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
            while (//opModeIsActive() &&
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
                Thread.yield();
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
                              double angle, Telemetry telemetry) throws InterruptedException {

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
        //if (opModeIsActive())
        {


            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);

            // keep looping while we are still active, and BOTH motors are running.
            while (//opModeIsActive() &&
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
                Thread.yield();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

        }
    }

    public String getcolor(int blue, int red, int green, Telemetry telemetry)
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
                                     double angle, Telemetry telemetry) throws InterruptedException {

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
        //if (opModeIsActive())
        {

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
            while (//opModeIsActive() &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                for (int i = 0; i< 2; i++) {

                    beaconcolor = getcolor(robot.sensor.blue(), robot.sensor.red(), robot.sensor.green(), telemetry);
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
//                    sleep(2400);
                    robot.sensorMotor.setPower(0.1);
//                    sleep(2400);
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
                Thread.yield();
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

    /*
    //---------------------------------------------------------------------------------------------------------------
    @Override
    public void runOpMode()  throws InterruptedException {

    }
*/
    public void Init(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {
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

        gyro.calibrate();

        // make sure the gyro is calibrated before continuing

        while (gyro.isCalibrating())  {
            Thread.yield();
            //idle();
        }
        telemetry.addData(">", "Calibrated Gyro");    //
        telemetry.update();

        gyro.resetZAxisIntegrator();

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData(">", "Robot Ready.");    //
    }
}
