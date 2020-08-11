package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Sensor Test", group = "Sensor")

public class DistanceSensorTest extends LinearOpMode {

    // Robot objects
    DistanceSensor sensorRange;
    ServoSensorHardware robot = new ServoSensorHardware();

    // Methods to speed up coding, not necessary
    public void say(String s) {
        telemetry.addLine(s);
    }
    public void update() {
        telemetry.update();
    }
    public double distance(DistanceUnit du) {
        return sensorRange.getDistance(du);
    }

    public void runOpMode() {
        // Initialize hardware
        robot.init(hardwareMap);
        sensorRange = robot.sensorRange;
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        // Initialize variables
        String[] valueKey = {"Distance (mm)"};

        // Wait for start
        telemetry.addLine("Press start to run the program.");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            // Post info to screen
            double[] valuePair = {distance(DistanceUnit.MM)};
            for (int i = 0; i < valueKey.length; i++) {
                say(valueKey[i] + ": " + valuePair[i]);
            }
            update();
        }

    }
}
