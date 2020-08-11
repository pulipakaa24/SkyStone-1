package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test Driving Teleop - 4 directions only", group = "Driving")
public class testTeleop  extends OpMode {

    // Names are acronyms for motor powers - f  = front, b = back, l = left, r = right, MP = motor power
    // Ex. flMP = front left motor power

    private double flMP, frMP, blMP, brMP = 0;

    ServoSensorHardware robot = new ServoSensorHardware();

    private boolean validMotorPower(double a) {
        return (a <= 1 && a >= -1);
    }

    private void say(String s) { telemetry.addLine(s);    }
    private void update() { telemetry.update(); }
    private void setPower(DcMotor m, double power) {
        m.setPower(power);
        say("Power of "+m.getDeviceName() + ": "+power);
    }
    private void power(double a, double b, double c, double d) {
       if(validMotorPower(a)) {
           flMP = a;
       }
        if(validMotorPower(b)) {
            frMP = b;
        }
        if(validMotorPower(c)) {
            blMP = c;
        }
        if(validMotorPower(d)) {
            brMP = d;
        }
    }

    public void init() {
        robot.init(hardwareMap);
    }
    public void loop() {
        if (gamepad1.dpad_up) {
            say("up");
            power(1,1,1,1);
        }
        else if(gamepad1.dpad_down) {
            say("down");
            power(-1,-1,-1,-1);
        }
        else if(gamepad1.dpad_right) {
            say("strafe right");
            power(1,-1,-1,1);
        }
        else if(gamepad1.dpad_left) {
            say("strafe left");
            power(-1,1,1,-1);
        }
        else if(gamepad1.b)  {
            say("rotate right");
            power(1,-1,1,-1);
        }
        else if(gamepad1.x)  {
            say("rotate left");
            power(-1,1,-1,1);
        }
        else {
            say("none");
            power(0,0,0,0);
        }
        setPower(robot.motor1, flMP);
        setPower(robot.motor2, frMP);
        setPower(robot.motor3, blMP);
        setPower(robot.motor4, brMP);
        update();
    }
}
