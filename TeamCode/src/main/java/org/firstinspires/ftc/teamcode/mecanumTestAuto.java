package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="BlueDetection", group="Bernard")
//@Disabled??'
public class mecanumTestAuto extends MecanumDriving{
    public void runOpMode()
    {
        super.runOpMode();
        float moveAmount;

        //robot.servoClaw.setPosition(0.5);
        robot.servoLatch.setPosition(.7);
        robot.servoTwist.setPosition(-1);
        sleep(500);
        robot.servoTwist.setPosition(0);

        mecanumEncoder(0.5, -5, -5.5, 5.5, "vertical");
        mecanumEncoder(1.7, -3, -3, 2, "lateral");


        moveAmount = skystoneDetection(1);

//        mecanumEncoder(0.5, 5, 5, 5, "vertical");
        mecanumTurn(1, 90, 5);
        gyroDrive(90, "x", .5, .4);
//        mecanumEncoder(1.7, -6, -6, 1, "lateral");
        sleep(500);
        mecanumEncoder(.5, -35 + moveAmount, -35 + moveAmount, 5, "vertical");
        sleep(500);
        //robot.servoClaw.setPosition(.5);
        skyLift(.5, 3, 3);
        sleep (500);
        skyLift(.5, -3, 3);

        mecanumEncoder(.7, 11, 11, 3, "vertical");
//        mecanumEncoder(1.0, -35, -35, 5, "vertical");

        telemetry.addData("Running", "Past the drive");
        telemetry.update();

    }
}