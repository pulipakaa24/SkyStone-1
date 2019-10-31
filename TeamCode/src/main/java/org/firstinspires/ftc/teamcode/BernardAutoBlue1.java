package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Blue!", group="Bernard")
public class BernardAutoBlue1 extends TestDriving {
    public void runOpMode()
    {
        super.runOpMode();

        while (robot.distSensor.getDistance(DistanceUnit.INCH) > 5)
        {
            encoderDrive(1, -3, -3, 5);//I'm not sure if negative is foward or backward
        }
        while (robot.sensorColor2.red() > robot.sensorColor2.blue() + 15 && robot.sensorColor2.green() > robot.sensorColor2.blue() + 15
                && robot.sensorColor.red() > robot.sensorColor.blue() + 15 && robot.sensorColor.green() > robot.sensorColor.blue() + 15)//this tests if a block is yellow. If you run this where there is no block, it'll likely assume that it is a skystone
        {
            latEncoderDrive(1, 2, 2, 3);//this, too, may be going the wrong way
        }
        //-------------grab the block.
        encoderDrive(1, 3,  3, 5);
        latEncoderDrive(1, -60, -60, 10);

    }
}