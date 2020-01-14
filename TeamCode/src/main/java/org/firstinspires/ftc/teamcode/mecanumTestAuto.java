package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="MecanumTestAuto", group="Bernard")
//@Disabled??'
public class mecanumTestAuto extends MecanumDriving{
    public void runOpMode()
    {
        super.runOpMode();
        float moveAmount;

        robot.servoClaw.setPosition(0);
        robot.servoTwist.setPosition(0);
        mecanumEncoder(0.5, -5, -5.5, 5.5, "vertical");

        moveAmount = skystoneDetection(1);

//        mecanumEncoder(0.5, 5, 5, 5, "vertical");
        mecanumTurn(1, 90, 5);
        mecanumEncoder(1, -33 + moveAmount, -33 + moveAmount, 5, "vertical");
//        sleep(100);
//        mecanumEncoder(1.0, -35, -35, 5, "vertical");

        telemetry.addData("Running", "Past the drive");
        telemetry.update();

    }
}