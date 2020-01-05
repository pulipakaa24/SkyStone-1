package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="MecanumTestAuto", group="Bernard")
//@Disabled
public class mecanumTestAuto extends MecanumDriving{
    public void runOpMode()
    {
        super.runOpMode();

        robot.servoClaw.setPosition(0);
        robot.servoTwist.setPosition(0);

        mecanumEncoder(0.5, -9, -9, 5, "vertical");
        while(opModeIsActive() && skystoneDetection(-1) != true)
        {
            skystoneDetection(-1);
        }

        mecanumEncoder(0.5, 5, 5, 5, "vertical");


        telemetry.addData("Running", "Past the drive");
        telemetry.update();
    }
}