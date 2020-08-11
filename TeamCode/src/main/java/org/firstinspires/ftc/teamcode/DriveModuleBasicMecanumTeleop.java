package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This file provides basic Telop driving for a Viridian Drive Module.

 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the DriveModuleHardware class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a Viridian Drive Module
 */

@TeleOp(name="Drive Module Teleop", group="DriveModule")

public class DriveModuleBasicMecanumTeleop extends OpMode{
    double FLpower = 0;
    double FRpower = 0;
    double RLpower = 0;
    double RRpower = 0;

    // Commit power values to motors
    void updatePowers() {
        robot.m0.setPower(FLpower);
        robot.m1.setPower(FRpower);
        robot.m2.setPower(RLpower);
        robot.m3.setPower(RRpower);
    }

    // Set all motor powers to 0
    void clearPowers() {
        FLpower = 0;
        FRpower = 0;
        RLpower = 0;
        RRpower = 0;
        updatePowers();
    }

    /* Declare OpMode members. */
    DriveModuleHardware robot       = new DriveModuleHardware(); // use the class created to define a Pushbot's hardware

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
            telemetry.addData("Say", "init() finished");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /** Stolen from https://ftcforum.firstinspires.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
         * Explained at https://www.roboteq.com/applications/all-blogs/5-driving-mecanum-wheels-omnidirectional-robots
         */

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        FLpower = r * Math.cos(robotAngle) + rightX;
        FRpower = r * Math.sin(robotAngle) - rightX;
        RLpower = r * Math.sin(robotAngle) + rightX;
        RRpower = r * Math.cos(robotAngle) - rightX;

        if (gamepad1.right_bumper) {
            FLpower /= 2;
            FRpower /= 2;
            RLpower /= 2;
            RRpower /= 2;
        }

        updatePowers();

        // Send telemetry message to signify robot running;
        telemetry.addData("Front left motor power",  "%.2f", FLpower);
        telemetry.addData("Front right motor power",  "%.2f", FRpower);
        telemetry.addData("Rear left motor power",  "%.2f", RLpower);
        telemetry.addData("Rear right motor power",  "%.2f", RRpower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        clearPowers();
    }
}
