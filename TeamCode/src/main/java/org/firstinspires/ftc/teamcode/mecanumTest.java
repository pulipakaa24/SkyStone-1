package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="MecanumTest", group="Bernard")
//@Disabled
public class mecanumTest extends MecanumDriving{
    public void runOpMode()
    {
        super.runOpMode();
        encoderDrive(10, "f", 5, 1);


    }
}