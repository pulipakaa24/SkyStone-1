//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class motorTestHardware
//{
//    // Here we define all of the motors we use.
//    // Our robot uses omni-wheels and a four motor drive system.
//
//    public DcMotor motorRight;
//
//
//
//
//
//    // Define hardware map
//    HardwareMap hwMap;
//
//    // Initialize standard hardware interfaces
//    public void init(HardwareMap ahwMap) {
//
//        hwMap = ahwMap;
//
//        // Here we name the motors.
//
//
//
//        motorRight = hwMap.get(DcMotor.class, "motorRight");
//
//
//
//
//
//        // Here we set the motor directions.
//        motorRight.setDirection(DcMotor.Direction.FORWARD);
//
//
//
//
//        // Here we set the servo directions.
//
//        // Here we set all motors to zero power for safety.
//
//        motorRight.setPower(0);
//        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }
//}