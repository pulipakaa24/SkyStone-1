package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Blue2", group="Bernard")
public class BernardAutoBlue2 extends TestDriving {
    public void runOpMode()
    {
        super.runOpMode();
        encoderDrive(.6,57,57,5);
        sleep(1000);
        robot.servoDrag.setPosition(.1);
        sleep(1000);
        encoderDrive(100,-50,-50,6);
        robot.servoLatch.setPosition(1);

    }
}