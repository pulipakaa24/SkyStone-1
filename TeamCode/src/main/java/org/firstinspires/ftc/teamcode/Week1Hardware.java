package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Week1Hardware {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;

    public static final double MAX_POWER = 0.99;

    HardwareMap hwMap =  null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftDrive  = hwMap.get(DcMotor.class, "motor_l");
        rightDrive  = hwMap.get(DcMotor.class, "motor_r");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
