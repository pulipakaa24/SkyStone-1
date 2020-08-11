package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Test", group = "Sensor")

public class ColorSensorTest extends LinearOpMode {

    // Robot objects
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    ServoSensorHardware robot = new ServoSensorHardware();

    // Methods to speed up coding, not necessary
    public void say(String s) {
        telemetry.addLine(s);
    }
    public void update() {
        telemetry.update();
    }
    public int distance() {
        int d = (int)sensorDistance.getDistance(DistanceUnit.MM);
        return d;
    }
    public int red() {
        int r = sensorColor.red();
        if (r < 0) {
            r = 0;
        }
        if (r > 255) {
            r = 255;
        }
        return r;
    }
    public int green() {
        int g = sensorColor.green();
        if (g < 0) {
            g = 0;
        }
        if (g > 255) {
            g = 255;
        }
        return g;
    }
    public int blue() {
        int b = sensorColor.blue();
        if (b < 0) {
            b = 0;
        }
        if (b > 255) {
            b = 255;
        }
        return b;
    }
    public int alpha() {
        int a = sensorColor.alpha();
        if (a < 0) {
            a = 0;
        }
        if (a > 255) {
            a = 255;
        }
        return a;
    }

    public void runOpMode() {
        // Initialize hardware
        robot.init(hardwareMap);
        sensorColor = robot.sensorColor;
        sensorDistance = robot.sensorDistance;

        // Initialize variables
        String[] valueKey = {"Distance (mm)","Red","Green","Blue","Alpha"};

        // Wait for start
        telemetry.addLine("Press start to run the program.");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            // Post colors to screen
            int[] valuePair = {distance(), red(), green(), blue(), alpha()};
            for (int i = 0; i < valueKey.length; i++) {
                say(valueKey[i] + ": " + valuePair[i]);
            }
            update();
        }

    }
}
