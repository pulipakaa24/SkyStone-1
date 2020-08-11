package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="foundation", group="Bernard")
//@Disabled??'
public class Foundation extends MecanumDriving{
    public void runOpMode()
    {
        super.runOpMode();
        float moveAmount;

        //robot.servoClaw.setPosition(0);
        robot.servoTwist.setPower(0);
        mecanumEncoder(0.5, -8, -8, 5.5, "vertical");
        skyLift(.5, 1, 3);
        sleep (500);
        skyLift(.5, -1, 3);
        mecanumEncoder(1.0, 5, 5, 5, "vertical");
        
//        mecanumEncoder(1.0, -35, -35, 5, "vertical");

        telemetry.addData("Running", "Past the drive");
        telemetry.update();

    }
}