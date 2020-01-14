package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaSkyStoneNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


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

    //VUFORIA STUFF SKYSTONE DETECTION
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY =
            "AQU7a8H/////AAABmfH4ZcQHIkPTjsjCf80CSVReJtuQBMiQodPHMSkdFHY8RhKT4fIEcY3JbCWjXRsUBFiewYx5etup17dUnX/SIQx6cjctrioEXrID+gV4tD9B29eCOdFVgyAr+7ZnEHHDYcSnt2pfzDZyMpi+I3IODqbUgVO82UiaZViuZBnA3dNvokZNFwZvv8/YDkcd4LhHv75QdkqgBzKe/TumwxjR/EqtR2fQRy9WnRjNVR9fYGl9MsuGNBSEmmys6GczXn8yZ/k2PKusiYz7h4hFGiXmlVLyikZuB4dxETGoqz+WWYUFJAdHzFiBptg5xXaa86qMBYBi3ht0RUiBKicLJhQZzLG0bIEJZWr198ihexUuhhGV";

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;


    //MOTOR CONSTANTS
    static final double COUNTS_PER_MOTOR_REV = 537; //216
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //static final double COUNTS_PER_INCH = 20;
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.5;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    VuforiaTrackables targetsSkyStone;
    VuforiaTrackable stoneTarget;
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

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuParameters.cameraName = webcamName;

        vuforia = ClassFactory.getInstance().createVuforia(vuParameters);
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");


        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");


        allTrackables.addAll(targetsSkyStone);
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, vuParameters.cameraDirection);
        }

        targetsSkyStone.activate();
        //tf.start(); //moved to start of program

    }

    public static double counts(double inches) {
        double newInches = (inches - 3.7959) / 1.1239;
        return newInches;
    }


    public void mecanumEncoder(double speed, double leftInches, double rightInches, double timeoutS, String direction) {
        int FLTarget = 0;
        int FRTarget = 0;
        int BLTarget = 0;
        int BRTarget = 0;
        /*telemetry.addData("currentposL: ", robot.motorLeft.getTargetPosition());
        telemetry.addData("currentposR: ", robot.motorRight.getTargetPosition());
        telemetry.update();*/

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            if (direction.equals("vertical")) {
                FLTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                FRTarget = robot.motorFrontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                BLTarget = robot.motorBackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                BRTarget = robot.motorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            } else if (direction.equals("lateral")) {
                FLTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                FRTarget = robot.motorFrontRight.getCurrentPosition() - (int) (rightInches * COUNTS_PER_INCH);
                BLTarget = robot.motorBackLeft.getCurrentPosition() - (int) (leftInches * COUNTS_PER_INCH);
                BRTarget = robot.motorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            } else {
                telemetry.addLine("not a valid direction");
                telemetry.update();
            }

            double correction = checkDirection();
            robot.motorFrontLeft.setTargetPosition(FLTarget);
            robot.motorFrontRight.setTargetPosition(FRTarget);
            robot.motorBackLeft.setTargetPosition(BLTarget);
            robot.motorBackRight.setTargetPosition(BRTarget);


            // Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFrontLeft.setPower(Math.abs(speed) - correction);
            robot.motorFrontRight.setPower(Math.abs(speed) + correction);
            robot.motorBackLeft.setPower(Math.abs(speed) - correction);
            robot.motorBackRight.setPower(Math.abs(speed) + correction);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())) {

                //Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", FLTarget, FRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);
            telemetry.addData("Path1", "Running to %7d :%7d", FLTarget, FRTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.motorFrontLeft.getCurrentPosition(),
                    robot.motorFrontRight.getCurrentPosition());
            telemetry.update();
            /*telemetry.addData("FinalposL: ", robot.motorLeft.getTargetPosition());
            telemetry.addData("FinalposR: ", robot.motorRight.getTargetPosition());
            telemetry.update();*/

            // Turn off RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }


    public void normalDrive(double lpower, double rpower) {

        if (opModeIsActive()) {
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorFrontRight.setPower(rpower);
            robot.motorFrontLeft.setPower(lpower);

            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.motorBackRight.setPower(rpower);
            robot.motorBackLeft.setPower(lpower);
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

    private double checkDirection() {
        double correction, angle, gain = .10;
        angle = getAngle();
        if (angle == 0)
            correction = 0;
        else
            correction = -angle;
        correction = correction * gain;
        return correction;
    }

    private double getAngle() {
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
    public float skystoneAlign() //This should automatically laterally align the robot to grab the block
    {

        while (!targetVisible) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    telemetry.update();

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }//Using Vuforia to search for the block. Stops once a block is found
        }
        VectorF translation = lastLocation.getTranslation();//initializes the "Translation." This allows us to see where the block is in relation to the robot.
        float moveAmount = -translation.get(1) /mmPerInch - 5.5f;//this should center the block and the grabbing mechanism. If it doesn't, try adjusting the -2
//            negative goes right? positive goes left    -6
        mecanumEncoder(1.7, moveAmount, moveAmount, 1, "lateral");
        return moveAmount;//this will allow us to keep track of how far the robot moves in the future.
    }
    public float skystoneDetection(int direction) {

        int speedMod = 4;
        float moveAmount = 0;
        boolean isFound = false;

        //!isStopRequested()
        while (isFound == false) {

            targetVisible = false;

            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    telemetry.update();

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
//            targetVisible = true; //this NEEDS to be committed out
            if (targetVisible) {
                isFound = true;
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                telemetry.addData("isFound: ", isFound);
                telemetry.update();
                sleep(500);
//                mecanumEncoder(0.5, 5*direction, 5*direction, 1, "lateral");
                moveAmount += skystoneAlign();
                sleep(500);
                mecanumEncoder(1.8, -6, -6, 5, "vertical");
                sleep(500);
                robot.servoClaw.setPosition(1);
                sleep(500);
                mecanumEncoder(0.9, 5, 5, 5, "vertical");

            } else {
                isFound = false;
                telemetry.addData("Visible Target", "none");
                telemetry.addData("isFound: ", isFound);
                telemetry.update();
                mecanumEncoder(0.5, speedMod*direction, speedMod*direction, 1, "lateral");
                sleep(100);
                moveAmount += speedMod * direction;
            }

        }

        targetsSkyStone.deactivate();
        return moveAmount;
    }

    public void mecanumTurn(double speed, double degrees, double timeoutS) {
        degrees = degrees / 90 * 10.5; //this was inches before. I made it so that you input degrees, then the program converts it to inches.
        int FLTarget = 0;//9.5
        int FRTarget = 0;
        int BLTarget = 0;
        int BRTarget = 0;
        /*telemetry.addData("currentposL: ", robot.motorLeft.getTargetPosition());
        telemetry.addData("currentposR: ", robot.motorRight.getTargetPosition());
        telemetry.update();*/

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            FLTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
            FRTarget = robot.motorFrontRight.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);
            BLTarget = robot.motorBackLeft.getCurrentPosition() + (int) (degrees * COUNTS_PER_INCH);
            BRTarget = robot.motorBackRight.getCurrentPosition() - (int) (degrees * COUNTS_PER_INCH);

            double correction = checkDirection();
            robot.motorFrontLeft.setTargetPosition(FLTarget);
            robot.motorFrontRight.setTargetPosition(FRTarget);
            robot.motorBackLeft.setTargetPosition(BLTarget);
            robot.motorBackRight.setTargetPosition(BRTarget);


            // Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFrontLeft.setPower(Math.abs(speed) - correction);
            robot.motorFrontRight.setPower(Math.abs(speed) + correction);
            robot.motorBackLeft.setPower(Math.abs(speed) - correction);
            robot.motorBackRight.setPower(Math.abs(speed) + correction);


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())) {

                //Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", FLTarget, FRTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motorFrontLeft.getCurrentPosition(),
                        robot.motorFrontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);
            telemetry.addData("Path1", "Running to %7d :%7d", FLTarget, FRTarget);
            telemetry.addData("Path2", "Running at %7d :%7d",
                    robot.motorFrontLeft.getCurrentPosition(),
                    robot.motorFrontRight.getCurrentPosition());
            telemetry.update();
            /*telemetry.addData("FinalposL: ", robot.motorLeft.getTargetPosition());
            telemetry.addData("FinalposR: ", robot.motorRight.getTargetPosition());
            telemetry.update();*/

            // Turn off RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }

    public void gyroDrive(double target, String xyz, double topPower, double timeoutS) {
        //Write code to correct to a target position (NOT FINISHED)

        runtime.reset();

        double angle = readAngle(xyz); //variable for gyro correction around z axis
        double error = angle - target;
        double powerScaled = topPower;
        do {
            angle = readAngle(xyz);
            error = angle - target;
            powerScaled = topPower * (error / 180) * pidMultiplier(error);


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
                        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorFrontLeft.setPower(powerScaled);
                        robot.motorFrontRight.setPower(-powerScaled);

                        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorBackLeft.setPower(powerScaled);
                        robot.motorBackRight.setPower(-powerScaled);
                    }
                }
            } else if (error < 0) {
                if (xyz.equals("z")) {
                    normalDrive(powerScaled, -powerScaled);
                }
                if (xyz.equals("y")) {
                    if (opModeIsActive()) {
                        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorFrontLeft.setPower(powerScaled);
                        robot.motorFrontRight.setPower(-powerScaled);

                        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.motorBackLeft.setPower(powerScaled);
                        robot.motorBackRight.setPower(-powerScaled);
                    }
                }
            }
//(Math.abs(0-error)>.3)
            //(error > 0.3 && error > 0) || (error < -0.3 && error < 0)
        } while (opModeIsActive() && ((error > 0.3) || (error < -0.3)) && (runtime.seconds() < timeoutS));
        normalDrive(0, 0);

    }

}