package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="motorTest", group="Bernard")
public class motorTest extends motorTestDriving {
    public void runOpMode()
    {
        super.runOpMode();
        robot.motorRight.setPower(1);
        sleep(10000);//test new branch
    }
}