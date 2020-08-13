package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Week1Hardware {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    public ColorSensor sensorColor;
    public DistanceSensor sensorDistance;
    public DigitalChannel digitalTouch;

    public static final double MAX_POWER = 1;

    HardwareMap hwMap           =  null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftDrive  = hwMap.get(DcMotor.class, "motor_l");
        rightDrive  = hwMap.get(DcMotor.class, "motor_r");

        sensorColor = hwMap.get(ColorSensor.class, "color_distance_sensor");
        digitalTouch = hwMap.get(DigitalChannel.class, "touch_sensor");
        sensorDistance = hwMap.get(DistanceSensor.class, "color_distance_sensor");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
    }
}