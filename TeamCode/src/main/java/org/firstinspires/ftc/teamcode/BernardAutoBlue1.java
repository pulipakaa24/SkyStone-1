package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Blue1", group="Bernard")
public class BernardAutoBlue1 extends TestDriving {
    public void runOpMode()
    {
        int moveAmount = 0;
        super.runOpMode();
        robot.servoGrab.setPosition(90);
        encoderDrive(1, -22, -22, 5);

        while (robot.distSensor.getDistance(DistanceUnit.INCH) > 2)
        {
            encoderDrive(1, -7, -7, .7);//I'm not sure if negative is forward or backward
        }
        robot.servoDrag.setPosition(1);
        while(robot.sensorColor.red() > robot.sensorColor.blue() + 15 && robot.sensorColor.green() > robot.sensorColor.blue() + 15)//this tests if a block is yellow. If you run this where there is no block, it'll likely assume that it is a skystone
        {
            latEncoderDrive(1, 2, 2, 1);
            moveAmount++;
        }
        robot.servoGrab.setPosition(0);
        encoderDrive(1, 3,  3, 5);
        gyroDrive(90, "z", 1, 5, false);


        encoderDrive(1, -60, -60, 10);
        robot.servoGrab.setPosition(90);

        gyroDrive(179, "z", 1, 2, false);
        encoderDrive(1, -86, -86, 10);
        gyroDrive(0, "z", 1, 5,false);
        encoderDrive(1, -3, -3, 5);

        robot.servoGrab.setPosition(0);
        encoderDrive(1, 3, 3, 2);
        gyroDrive(90, "z", 1, 4,false);
        encoderDrive(1,-86, -86, 10);
        robot.servoGrab.setPosition(90);
        gyroDrive(179, "z", 1, 3, false);
        encoderDrive(1, -2 * moveAmount, -2*moveAmount, 10);

    }
}