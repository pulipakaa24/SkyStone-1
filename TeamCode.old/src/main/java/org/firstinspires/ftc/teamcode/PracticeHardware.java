package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PracticeHardware
{
    // Here we define all of the motors we use.
    // Our robot uses omni-wheels and a four motor drive system.
    public DcMotor motorFrontLeft;
    public DcMotor motorBackRight;
    public DcMotor motorBackLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorLift;
    public Servo servoClaw;
    public Servo servoTwist;
    public Servo servoDrag;




    // Define hardware map
    HardwareMap hwMap;

    // Initialize standard hardware interfaces
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        //Here we name the servos
        servoClaw = hwMap.get(Servo.class, "servoClaw");
        servoTwist = hwMap.get(Servo.class,"servoTwist");
        servoDrag = hwMap.get(Servo.class,"servoDrag");

        // Here we name the motors.
        motorFrontLeft  = hwMap.get(DcMotor.class, "motorFrontLeft");
        motorBackRight = hwMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft  = hwMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight  = hwMap.get(DcMotor.class, "motorFrontRight");

        motorLift  = hwMap.get(DcMotor.class, "motorLift");


        //Here we set the servo directions
        servoClaw.setDirection(Servo.Direction.FORWARD);
        servoTwist.setDirection(Servo.Direction.REVERSE);
        servoDrag.setDirection(Servo.Direction.REVERSE);


        // Here we set the motor directions.
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorLift.setDirection(DcMotor.Direction.REVERSE);


        // Here we set all motors to zero power for safety.
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorLift.setPower(0);


        //servoClaw.scaleRange(0,0.5);
        servoClaw.setPosition(0.0);
        servoTwist.setPosition(0.0);
        servoDrag.setPosition(0.9);



        // Here we set all motors to run with encoders.
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}