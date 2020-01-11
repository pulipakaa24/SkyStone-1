package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="MecanumTest", group="Bernard")
//@Disabled
public class mecanumTest extends MecanumDriving{
    public void runOpMode()
    {


        super.runOpMode();


        mecanumTurn(1.0, 9.5, 5);
        telemetry.addData("Running", "Past the drive");
        telemetry.update();
    }
}