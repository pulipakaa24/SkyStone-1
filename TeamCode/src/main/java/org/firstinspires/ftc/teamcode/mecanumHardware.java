package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class mecanumHardware
{
    // Here we define all of the motors we use.
    // Our robot uses omni-wheels and a four motor drive system.
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorLiftR;
    public DcMotor motorLiftL;
    public Servo servoGrab;
    //public Servo servoDrag;
//    public DcMotor leftLift;
//    public DcMotor rightLift;






    // Define hardware map
    HardwareMap hwMap;

    // Initialize standard hardware interfaces
    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Here we name the motors.
        motorFrontLeft  = hwMap.get(DcMotor.class, "motorFrontLeft");


        motorBackRight = hwMap.get(DcMotor.class, "motorBackRight");


        motorBackLeft  = hwMap.get(DcMotor.class, "motorBackLeft");


        motorFrontRight  = hwMap.get(DcMotor.class, "motorFrontRight");

        motorLiftR = hwMap.get(DcMotor.class,"motorLiftRight");
        motorLiftL = hwMap.get(DcMotor.class, "motorLiftLeft");

        //servoDrag = hwMap.get(Servo.class, "servoDrag");
//        leftLift  = hwMap.get(DcMotor.class, "leftLift");
//
//
//        rightLift  = hwMap.get(DcMotor.class, "rightLift");



        //90 is up | 0 is down

        servoGrab = hwMap.get(Servo.class, "servoGrab");



        // Here we set the motor directions.
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
//        leftLift.setDirection(DcMotor.Direction.FORWARD);
//        rightLift.setDirection(DcMotor.Direction.REVERSE);



        // Here we set the servo directions.

        // Here we set all motors to zero power for safety.
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorLiftR.setPower(0);
        motorLiftL.setPower(0);

        servoGrab.setPosition(0.5);
        //servoDrag.setPosition(.9);
//        leftLift.setPower(0);
//        rightLift.setPower(0);


        // Here we set all motors to run with encoders.
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLiftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}