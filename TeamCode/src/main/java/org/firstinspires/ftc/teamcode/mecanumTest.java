package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="MecanumTest", group="Bernard")
//@Disabled
public class mecanumTest extends MecanumDriving{
    public void runOpMode()
    {


        super.runOpMode();

        mecanumEncoder(1.0, -8, -8, 10, "vertical");
        mecanumEncoder(1.0, 8, 8, 10, "vertical");
        mecanumEncoder(1.0, -8, -8, 5, "lateral");
        mecanumEncoder(1.0, 8, 8, 5, "lateral");

        telemetry.addData("Running", "Past the drive");
        telemetry.update();
    }
}