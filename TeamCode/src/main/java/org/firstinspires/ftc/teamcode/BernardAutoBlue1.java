package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Blue1", group="Bernard")
public class BernardAutoBlue1 extends TestDriving {
    public void runOpMode()
    {
        int moveAmount = 0;
        super.runOpMode();
        robot.servoTwist.setPosition(0);
        robot.servoGrab.setPosition(-45);
        robot.servoLatch.setPosition(1);
        encoderDrive(.3,-30,-30,20);
        while (robot.distSensor.getDistance(DistanceUnit.INCH) > 4)
        {
            normalDrive(-.2,-.2);
            //encoderDrive(1, -3, -3, .7);//I'm not sure if negative is forward or backward
        }
        //((robot.sensorColor.alpha() + robot.sensorColor2.alpha())/2)>150
        boolean found = false;
        sleep(100);
        while (!found)
        {
            sleep(1000);
            if (isSkystone())
            {
                break;
            }
            else
                {
                    robot.motorBack.setPower(.3);
                    robot.motorFront.setPower(.3);
                }
            moveAmount++;
            sleep(300);
        }
        robot.motorFront.setPower(0);
        robot.motorFront.setPower(0);
        //while(robot.sensorColor.red() > robot.sensorColor.blue() + 15 && robot.sensorColor.green() > robot.sensorColor.blue() + 15)//this tests if a block is yellow. If you run this where there is no block, it'll likely assume that it is a skystone
        robot.servoGrab.setPosition(90);
        telemetry.addData("Color:","skystone");
        encoderDrive(1, 3,  3, 5);
        gyroDrive(-90, "z", 1, 5, false);


        encoderDrive(1, -60, -60, 10);
        robot.servoGrab.setPosition(0);

        gyroDrive(179, "z", 1, 2, false);
        encoderDrive(1, -86, -86, 10);
        gyroDrive(0, "z", 1, 5,false);
        encoderDrive(1, -3, -3, 5);

        robot.servoGrab.setPosition(90);
        encoderDrive(1, 3, 3, 2);
        gyroDrive(90, "z", 1, 4,false);
        encoderDrive(1,-86, -86, 10);
        robot.servoGrab.setPosition(0);
        gyroDrive(179, "z", 1, 3, false);
        encoderDrive(1, -2 * moveAmount, -2*moveAmount, 10);

    }
}