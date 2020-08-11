package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TestServo", group="Bernard")
public class testServo extends MecanumDriving {

    public void runOpMode() {
        super.runOpMode();

        //robot.servoClaw.setPosition(-1);
        sleep(500);
        //robot.servoClaw.setPosition(-.5);
        sleep(500);
        //robot.servoClaw.setPosition(0);
        sleep(500);

        //robot.servoClaw.setPosition(.5);
        sleep(500);


        //robot.servoClaw.setPosition(1);



    }
}
