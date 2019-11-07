package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TestHardware
{
    // Here we define all of the motors we use.
    // Our robot uses omni-wheels and a four motor drive system.
    public DcMotor motorLeft;
    public DcMotor motorRight;
    public DcMotor motorFront;
    public DcMotor motorBack;
    public DcMotor leftLift;
    public DcMotor rightLift;
    public Servo servoTwist;
    public Servo servoGrab;
    public Servo servoDrag;

    ColorSensor sensorColor = null;
    ColorSensor sensorColor2 = null;

    Rev2mDistanceSensor distSensor = null;




    // Define hardware map
    HardwareMap hwMap;

    // Initialize standard hardware interfaces
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Here we name the motors.
        motorLeft  = hwMap.get(DcMotor.class, "motorLeft");


        motorRight = hwMap.get(DcMotor.class, "motorRight");


        motorBack  = hwMap.get(DcMotor.class, "motorBack");


        motorFront  = hwMap.get(DcMotor.class, "motorFront");


        leftLift  = hwMap.get(DcMotor.class, "leftLift");


        rightLift  = hwMap.get(DcMotor.class, "rightLift");


        servoTwist = hwMap.get(Servo.class, "servoTwist");


        servoGrab = hwMap.get(Servo.class, "servoGrab");
        //

        servoDrag = hwMap.get(Servo.class, "servoDrag");
        //90 is up | 0 is down



        // Here we set the motor directions.
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFront.setDirection(DcMotor.Direction.FORWARD);
        motorBack.setDirection(DcMotor.Direction.REVERSE);
        leftLift.setDirection(DcMotor.Direction.FORWARD);
        rightLift.setDirection(DcMotor.Direction.REVERSE);



        // Here we set the servo directions.

        // Here we set all motors to zero power for safety.
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorFront.setPower(0);
        motorBack.setPower(0);
        leftLift.setPower(0);
        rightLift.setPower(0);

        servoTwist.setPosition(90);
        servoGrab.setPosition(90);
        servoDrag.setPosition(90);
        // Here we set all motors to run with encoders.
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sensorColor = hwMap.get(ColorSensor.class, "sensorColor");
        sensorColor2 = hwMap.get(ColorSensor.class, "sensorColor2");

        distSensor = hwMap.get(Rev2mDistanceSensor.class, "distanceSensor");

    }
}