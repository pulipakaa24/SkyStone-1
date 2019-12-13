package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="MecanumTest", group="Bernard")
//@Disabled
public class mecanumTest extends MecanumDriving{
    public void runOpMode()
    {
        telemetry.addData("Running", "pre-reunOp");
        telemetry.update();
        super.runOpMode();
        telemetry.addData("Running", "Just past runOp");
        telemetry.update();
        encoderDrive(2, "f", 2, 3);
        encoderDrive(10, "b", 10, 1);
        normalDrive(1, 1);
        telemetry.addData("Running", "Past the drive");
        telemetry.update();

    }
}