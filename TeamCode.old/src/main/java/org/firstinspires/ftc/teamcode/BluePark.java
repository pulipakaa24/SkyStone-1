package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BluePark", group="Bernard")
public class BluePark extends MecanumDriving{

    @Override
    public void runOpMode() {
        super.runOpMode();

        mecanumEncoder(1, 25, 25, 5, "lateral");
    }
}
