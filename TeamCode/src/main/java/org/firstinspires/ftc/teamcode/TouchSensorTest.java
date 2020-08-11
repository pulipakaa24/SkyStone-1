package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Touch Sensor Test", group = "Sensor")

public class TouchSensorTest extends LinearOpMode {

    // Robot objects
    RevTouchSensor digitalTouch;
    ServoSensorHardware robot = new ServoSensorHardware();

    // Methods to speed up coding, not necessary
    public void say(String s) {
        telemetry.addLine(s);
    }
    public void update() {
        telemetry.update();
    }
    public boolean pressed() {
        return digitalTouch.isPressed() ;
    }

    public void runOpMode() {
        // Initialize hardware
        robot.init(hardwareMap);
        digitalTouch = robot.digitalTouch;

        // Initialize variables
        String[] valueKey = {"Pressed"};

        // Wait for start
        telemetry.addLine("Press start to run the program.");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            // Post info to screen
            boolean[] valuePair = {pressed()};
            for (int i = 0; i < valueKey.length; i++) {
                say(valueKey[i] + ": " + valuePair[i]);
            }
            update();
        }

    }
}
