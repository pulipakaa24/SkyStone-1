package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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

public class TestDriving extends LinearOpMode {

    /* Declare OpMode members. */
    TestHardware robot = new TestHardware();
    private ElapsedTime runtime = new ElapsedTime();
    String xyz = "z";


    static final double COUNTS_PER_MOTOR_REV = 537; //216
    static final double DRIVE_GEAR_REDUCTION = 0.6666;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
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
        robot.motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //lateral motors
        robot.motorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        //tf.start(); //moved to start of program

    }

    public static double counts(double inches) {
        double newInches = (inches - 3.7959) / 1.1239;
        return newInches;
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        /*telemetry.addData("currentposL: ", robot.motorLeft.getTargetPosition());
        telemetry.addData("currentposR: ", robot.motorRight.getTargetPosition());
        telemetry.update();*/

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            double correction = checkDirection();
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorLeft.setPower(Math.abs(speed) - correction);
            robot.motorRight.setPower(Math.abs(speed) + correction);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLeft.isBusy() && robot.motorRight.isBusy())) {

                //Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motorLeft.getCurrentPosition(),
                        robot.motorRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.motorLeft.getCurrentPosition(),
                    robot.motorRight.getCurrentPosition());
            telemetry.update();
            /*telemetry.addData("FinalposL: ", robot.motorLeft.getTargetPosition());
            telemetry.addData("FinalposR: ", robot.motorRight.getTargetPosition());
            telemetry.update();*/

            // Turn off RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void latEncoderDrive(double speed, double frontInches, double backInches, double timeoutS) {
        int newFrontTarget;
        int newBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontTarget = robot.motorFront.getCurrentPosition() + (int) (frontInches * COUNTS_PER_INCH);
            newBackTarget = robot.motorBack.getCurrentPosition() + (int) (backInches * COUNTS_PER_INCH);
            robot.motorFront.setTargetPosition(newFrontTarget);
            robot.motorBack.setTargetPosition(newBackTarget);

            // Turn On RUN_TO_POSITION
            robot.motorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFront.setPower(Math.abs(speed));
            robot.motorBack.setPower(Math.abs(speed));

            // keep looping while we are sill active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            //i'm really hungry right now. just thought everyone programming should now that. -your captain, Luke
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFront.isBusy() && robot.motorBack.isBusy())) {

                // Display it for the driver.
               /* telemetry.addData("Path1",  "Running to %7d :%7d", newFrontTarget,  newBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorFront.getCurrentPosition(),
                        robot.motorBack.getCurrentPosition());
                telemetry.update();*/
            }

            // Stop all motion;
            robot.motorFront.setPower(0);
            robot.motorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void normalDrive(double lpower, double rpower) {

        if (opModeIsActive()) {
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorRight.setPower(rpower);
            robot.motorLeft.setPower(lpower);
        }
    }

    public boolean isSkystone()
    {
        sleep(100);
        if (robot.sensorColor.red() > robot.sensorColor.blue() + 15 && robot.sensorColor.green() > robot.sensorColor.blue() + 15)
        {
            telemetry.addData("Left Block Color", "Yellow");

        }
        else
            telemetry.addData("Left Block Color", "Not Yellow");
            boolean leftBlack = true;

        if (robot.sensorColor2.red() > robot.sensorColor2.blue() + 15 && robot.sensorColor2.green() > robot.sensorColor2.blue() + 15)
        {
            telemetry.addData("Right Block Color", "Yellow");
        }
        else
            telemetry.addData("Right Block Color", "Not Yellow");
            boolean rightBlack = true;
        telemetry.update();

        if (rightBlack && leftBlack)
        {
            return true;
        }
        else
            {
                return false;
            }
    }

    public double originalAngle;

    public void gyroDrive(double target, String xyz, double topPower, double timeoutS, boolean isCorrection) {
        //Write code to correct to a target position (NOT FINISHED)

        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle - target;
        double powerScaled = topPower;
        do {
            angle = readAngle(xyz);
            error = angle - target;
            if (!isCorrection) {
                powerScaled = topPower * (error / 180) * pidMultiplier(error);
            }

            //double powerScaled = power*pidMultiplier(error);
            telemetry.addData("original angle", originalAngle);
            telemetry.addData("current angle", readAngle(xyz));
            telemetry.addData("error", error);
            telemetry.update();
            if (error > 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("y")) {
                    if (opModeIsActive()) {
                        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorLeft.setPower(powerScaled);
                        robot.motorRight.setPower(powerScaled);
                    }
                }
            } else if (error < 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("y")) {
                    if (opModeIsActive()) {
                        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorLeft.setPower(powerScaled);
                        robot.motorRight.setPower(powerScaled);
                    }
                }
            }
//(Math.abs(0-error)>.3)
            //(error > 0.3 && error > 0) || (error < -0.3 && error < 0)
        } while (opModeIsActive() && ((error > 0.3) || (error < -0.3)) && (runtime.seconds() < timeoutS));
        normalDrive(0, 0);

    }

    public double pidMultiplier(double error) {
        //equation for power multiplier is x/sqrt(x^2 + C)
        int C = 600;
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

    public void correctionDrive(double speed, double power, double leftInches, double rightInches,
                                double timeoutS) {
        originalAngle = readAngle(xyz);
        double powerScaled = power * pidMultiplier(originalAngle);
        int newLeftTarget;
        int newRightTarget;
        double lastCorrection = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorLeft.setPower(Math.abs(speed));
            robot.motorRight.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLeft.isBusy() && robot.motorRight.isBusy())) {

                double angleDifference = Math.abs(readAngle(xyz)) - Math.abs(originalAngle);

                if (angleDifference > 3) {

                    gyroDrive(originalAngle, xyz, -power, timeoutS - getRuntime(), true);
//                    lastCorrection = getRuntime();
                    robot.motorLeft.setTargetPosition(newLeftTarget - robot.motorLeft.getCurrentPosition());
                    robot.motorRight.setTargetPosition(newRightTarget - robot.motorRight.getCurrentPosition());
                    robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.motorLeft.setPower(Math.abs(speed));
                    robot.motorRight.setPower(Math.abs(speed));
                }
                telemetry.addData("distance", newLeftTarget - robot.motorLeft.getCurrentPosition());
                telemetry.update();

            }

            // Stop all motion;
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
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

    public void oldGyroDrive(double target, String xyz, double power, double timeoutS)
    {
        //Write code to correct to a target position (NOT FINISHED)

        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle-target;
        double powerScaled = power*pidMultiplier(error);
        do{
            angle = readAngle(xyz);
            error = angle - target;
            powerScaled = power*pidMultiplier(error);
            telemetry.addData("error", error);
            telemetry.update();
            if(error > 0)
            {
                if(xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if(xyz.equals("y"))
                {
                    if (opModeIsActive()) {
                        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorLeft.setPower(powerScaled);
                        robot.motorRight.setPower(powerScaled);
                    }
                }
            }
            else if(error < 0)
            {
                if(xyz.equals("z")) {
                    normalDrive(-powerScaled, powerScaled);
                }
                if(xyz.equals("y"))
                {
                    if (opModeIsActive()) {
                        robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorLeft.setPower(powerScaled);
                        robot.motorRight.setPower(powerScaled);
                    }
                }
            }
//(Math.abs(0-error)>.3)
            //(error > 0.3 && error > 0) || (error < -0.3 && error < 0)
        }while(opModeIsActive() && ((error > 1) || (error < -1)) && (runtime.seconds() < timeoutS));
        normalDrive(0,0);

    }
}