package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Blue1", group="Bernard")
public class BernardAutoBlue1 extends TestDriving {
    public void runOpMode()
    {
        super.runOpMode();
        encoderDrive(1, -22, -22, 5);
        while (robot.distSensor.getDistance(DistanceUnit.INCH) > 2)
        {
            encoderDrive(1, -7, -7, 5);//I'm not sure if negative is forward or backward
        }
        robot.servoDrag.setPosition(1);
        for (int i = 0; i < 10; i++)
                //|| robot.sensorColor.red() > robot.sensorColor.blue() + 15 && robot.sensorColor.green() > robot.sensorColor.blue() + 15)//this tests if a block is yellow. If you run this where there is no block, it'll likely assume that it is a skystone
        {
            latEncoderDrive(.2, 2, 2, 3);
            robot.servoDrag.setPosition(1);
            robot.servoDrag.setPosition(-1);
        }
        //-------------grab the block.
        encoderDrive(1, 3,  3, 5);
        latEncoderDrive(1, -60, -60, 10);

    }
}