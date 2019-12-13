package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *
 *
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class MecanumDriving extends LinearOpMode {

    /* Declare OpMode members. */
    mecanumHardware robot = new mecanumHardware();
    private ElapsedTime runtime = new ElapsedTime();
    String xyz = "z";


    static final double COUNTS_PER_MOTOR_REV = 537; //216
    static final double DRIVE_GEAR_REDUCTION = 0.6666;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);
static final double COUNTS_PER_INCH = 20;
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.5;
    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //init distance sensors
        /*
        Rev2mDistanceSensor sensorRangeR = hardwareMap.get(Rev2mDistanceSensor.class, "sensorRangeR");
        Rev2mDistanceSensor sensorRangeL = hardwareMap.get(Rev2mDistanceSensor.class, "sensorRangeL");
        sensorRangeL.initialize();
        sensorRangeR.initialize();
        */
        //side motors
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lateral motors
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        //tf.start(); //moved to start of program

    }

    public static double counts(double inches) {
        double newInches = (inches - 3.7959) / 1.1239;
        return newInches;
    }

    public void encoderDrive(double inches, String direction, double timeoutS, double topPower) {


        telemetry.addData("Running", "IN THE THING AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
        telemetry.addData("Running", "IN THE THING AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
        telemetry.addData("Running", "IN THE THING AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
        telemetry.addData("Running", "IN THE THING AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
        telemetry.addData("Running", "IN THE THING AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
        telemetry.addData("Running", "IN THE THING AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
        telemetry.addData("Running", "IN THE THING AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");

        telemetry.update();
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(1000);
        int TargetFL = 0;
        int TargetFR = 0;
        int TargetBL = 0;
        int TargetBR = 0;
        double errorFL = 0;
        double errorFR = 0;
        double errorBL = 0;
        double errorBR = 0;
        double powerFL = 0;
        double powerFR = 0;
        double powerBL = 0;
        double powerBR = 0;


        String heading = direction;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            if (heading.equals("f")) {
                TargetFL = robot.motorFrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                TargetFR = robot.motorFrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                TargetBL = robot.motorBackLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                TargetBR = robot.motorBackRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                telemetry.addData("Running", "IN THE THING front");
                telemetry.addData("Running", "IN THE THING front");
                telemetry.addData("Running", "IN THE THING front");
                telemetry.addData("Running", "IN THE THING front");
                telemetry.addData("Running", "IN THE THING front");
                telemetry.addData("Running", "IN THE THING front");
                telemetry.addData("Running", "IN THE THING front");

                telemetry.update();

            } else if (heading.equals("b")) {
                TargetFL = robot.motorFrontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                TargetFR = robot.motorFrontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                TargetBL = robot.motorBackLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                TargetBR = robot.motorBackRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);


            } else if (heading.equals("r")) {
                TargetFL = robot.motorFrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                TargetFR = robot.motorFrontRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                TargetBL = robot.motorBackLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                TargetBR = robot.motorBackLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH); //weird should be +


            } else if (heading.equals("l")) {
                TargetFL = robot.motorFrontLeft.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);
                TargetFR = robot.motorFrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                TargetBL = robot.motorBackLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH); // weird should be +
                TargetBR = robot.motorBackRight.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH);

            } else {
                telemetry.addData("not a valid direction", heading);
                telemetry.update();
            }

            telemetry.addData("Front Left Current: ", robot.motorFrontLeft.getCurrentPosition());
            telemetry.addData("Front Right Current: ", robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("Back Left Current: ", robot.motorBackLeft.getCurrentPosition());
            telemetry.addData("Back Right Current: ", robot.motorBackRight.getCurrentPosition());
            telemetry.addData("", "");
            telemetry.addData("Front Left Target: ", TargetFL);
            telemetry.addData("Front Right Target: ", TargetFR);
            telemetry.addData("Back Left Target: ", TargetBL);
            telemetry.addData("Back Right Target: ", TargetBR);
            telemetry.update();

            sleep(1000);

            // Determine new target position, and pass to motor controller

//            robot.motorFrontLeft.setTargetPosition(TargetFL);
//            robot.motorFrontRight.setTargetPosition(TargetFR);
//            robot.motorBackLeft.setTargetPosition(TargetBL);
//            robot.motorBackRight.setTargetPosition(TargetBR);
//
//
//            // Turn On RUN_TO_POSITION
//            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontLeft.setPower(TargetFL);
            robot.motorFrontRight.setPower(TargetFR);
            robot.motorBackLeft.setPower(TargetBL);
            robot.motorBackRight.setPower(TargetBR);


            // reset the timeout time and start motion.
            runtime.reset();
            /*robot.fLMotor.setPower(Speed);
            robot.fRMotor.setPower(Speed);
            robot.bRMotor.setPower(Speed);
            robot.bLMotor.setPower(Speed);*/


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && ((robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy()) && robot.motorBackLeft.isBusy() && robot.motorBackRight.isBusy())) {
                errorFL = TargetFL - robot.motorFrontLeft.getCurrentPosition();
                errorFR = TargetFR - robot.motorFrontRight.getCurrentPosition();
                errorBL = TargetBL - robot.motorBackLeft.getCurrentPosition();
                errorBR = TargetBR - robot.motorBackRight.getCurrentPosition();

                powerFL = topPower * pidMultiplier(errorFL);
                powerFR = topPower * pidMultiplier(errorFR);
                powerBL = topPower * pidMultiplier(errorBL);
                powerBR = topPower * pidMultiplier(errorBR);

                robot.motorFrontLeft.setPower(Math.abs(powerFL));
                robot.motorFrontRight.setPower(Math.abs(powerFR));
                robot.motorBackRight.setPower(Math.abs(powerBL));
                robot.motorBackLeft.setPower(Math.abs(powerBR));
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", TargetFL, TargetFR, TargetBL, TargetBR);

                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d", robot.motorFrontLeft.getCurrentPosition(), robot.motorFrontRight.getCurrentPosition(), robot.motorBackLeft.getCurrentPosition(), robot.motorBackRight.getCurrentPosition());
                //telemetry.addData("speeds",  "Running to %7f :%7f :%7f :%7f", speedfL,  speedfR, speedfL, speedbR);
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move

            robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void normalDrive(double lpower, double rpower) {

        if (opModeIsActive()) {
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFrontRight.setPower(rpower);
            robot.motorFrontLeft.setPower(lpower);
        }
    }

    public double originalAngle;


    public double pidMultiplier(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        int C = 100;
        return Math.abs(error / Math.sqrt((error * error) + C));
    }

    public double readAngle(String xyz) {
        Orientation angles;
        Acceleration gravity;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (xyz.equals("x")) {
            return angles.thirdAngle;
        } else if (xyz.equals("y")) {
            return angles.secondAngle;
        } else if (xyz.equals("z")) {
            return angles.firstAngle;
        } else {
            return 0;
        }
    }

    public double getOffAngle() {
        return 5;
    }

    private double checkDirection()
    {
        double correction, angle, gain = .10;
        angle = getAngle();
        if (angle == 0)
            correction =0;
        else
            correction = -angle;
        correction = correction * gain;
        return correction;
    }
    private double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


}