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

//vuforia key:
//            "AQU7a8H/////AAABmfH4ZcQHIkPTjsjCf80CSVReJtuQBMiQodPHMSkdFHY8RhKT4fIEcY3JbCWjXRsUBFiewYx5etup17dUnX/SIQx6cjctrioEXrID+gV4tD9B29eCOdFVgyAr+7ZnEHHDYcSnt2pfzDZyMpi+I3IODqbUgVO82UiaZViuZBnA3dNvokZNFwZvv8/YDkcd4LhHv75QdkqgBzKe/TumwxjR/EqtR2fQRy9WnRjNVR9fYGl9MsuGNBSEmmys6GczXn8yZ/k2PKusiYz7h4hFGiXmlVLyikZuB4dxETGoqz+WWYUFJAdHzFiBptg5xXaa86qMBYBi3ht0RUiBKicLJhQZzLG0bIEJZWr198ihexUuhhGV";
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

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        // waitForStart();

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

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
        mecanumEncoder(1.0, moveAmount/2, moveAmount/2, 1, "lateral");
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
//                moveAmount += skystoneAlign();
                sleep(500);
                mecanumEncoder(1.8, -6, -6, 5, "vertical");
                sleep(500);
                //robot.servoClaw.setPosition(-1);
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
    public void skyLift(double speed, double inches, double timeoutS)// POSITIVE IS: up? NEGATIVE IS: down?
    {
//        int rLiftTarget = robot.motorLiftR.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
//        int lLiftTarget = robot.motorLiftR.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);//calculates how far the motor needs to move
//
//        robot.motorLiftR.setTargetPosition(rLiftTarget);//assigns position
//        robot.motorLiftL.setTargetPosition(lLiftTarget);
//
//        robot.motorLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);//changes to run a number of inches, rather than using encoders
//        robot.motorLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        runtime.reset();//resets the runtime so that the time out works
//
//        robot.motorLiftR.setPower(Math.abs(speed) + correction);//turns the motors on
//        robot.motorLiftL.setPower(Math.abs(speed) + correction);
//
//        while (opModeIsActive() &&//ends the motors when time runs out
//                (runtime.seconds() < timeoutS) &&
//                (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())){}
//
//        robot.motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//turns the motors back to encoders.
//        robot.motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//


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
        double error;
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
                if (xyz.equals("x")) {
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
                if (xyz.equals("x")) {
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
        } while (opModeIsActive() && ((error > 0.5) || (error < -0.5)) && (runtime.seconds() < timeoutS));
        normalDrive(0, 0);

    }

}