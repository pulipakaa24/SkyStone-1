package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Blue1", group="Bernard")
@Disabled
public class BernardAutoBlue1 extends TestDriving {
    public void runOpMode()
    {
//        int moveAmount = 0;
//        super.runOpMode();
//        robot.servoTwist.setPosition(0.5);
//        robot.servoGrab.setPosition(0);
//        robot.servoLatch.setPosition(1);
//        sleep(500);
//        gyroDrive(0, "z", -1, 3, false);
//        sleep(250);
//        encoderDrive(1,-30,-30,20);
//        while (robot.distSensor.getDistance(DistanceUnit.INCH) > 5)
//        {
//            normalDrive(-.2,-.2);
//            telemetry.addData("dist", robot.distSensor.getDistance(DistanceUnit.INCH));
//            //encoderDrive(1, -3, -3, .7);//I'm not sure if negative is forward or backward
//        }
//        normalDrive(0,0);
//        //((robot.sensorColor.alpha() + robot.sensorColor2.alpha())/2)>150
//        boolean found = false;
//        sleep(500);
//        /*
//        while (((robot.sensorColor.alpha() + robot.sensorColor2.alpha())/2) > 80)
//        {
//            robot.motorBack.setPower(.06);
//            robot.motorFront.setPower(.09);
//            moveAmount++;
//            sleep(1000);
//            robot.motorFront.setPower(0);
//            robot.motorBack.setPower(0);
//            telemetry.addData("avg",((robot.sensorColor.alpha() + robot.sensorColor2.alpha())/2));
//            telemetry.update();
//            sleep(200);
//
//        }
//
//
//        // sysdel(naren.exe)
//        robot.motorFront.setPower(0);
//        robot.motorFront.setPower(0);
//        //while(robot.sensorColor.red() > robot.sensorColor.blue() + 15 && robot.sensorColor.green() > robot.sensorColor.blue() + 15)//this tests if a block is yellow. If you run this where there is no block, it'll likely assume that it is a skystone
//        encoderDrive(1,-2,-2,5);
//        robot.servoGrab.setPosition(1);
//        telemetry.addData("Color:","skystone");
//        */
//        encoderDrive(1, 2,  2, 5);
//        robot.servoGrab.setPosition(1);
//        encoderDrive(1, -3,  -3, 5);
//        gyroDrive(105, "z", -1, 3, false);
//
//        encoderDrive(.8, -60, -60, 10);
//        encoderDrive(1, -40, -40, 10);
//        sleep(500);;
//
//        robot.leftLift.setPower(.5);
//        robot.leftLift.setPower(.5);
//        robot.servoGrab.setPosition(0);
//        encoderDrive(1,-3,-3,3);
//        encoderDrive(1,50,50,5);
//
////        gyroDrive(-140, "z", 1, 2, false);
////        encoderDrive(.6, -40, -40, 10);
////        encoderDrive(1, -30, -30, 10);
////        gyroDrive(0, "z", 1, 5,false);
////        encoderDrive(1, -3, -3, 5);
////
////        robot.servoGrab.setPosition(90);
////        encoderDrive(1, 3, 3, 2);
////        gyroDrive(90, "z", 1, 4,false);
////        encoderDrive(1,-86, -86, 10);
////        robot.servoGrab.setPosition(0);
////        gyroDrive(179, "z", 1, 3, false);
////        encoderDrive(1, -2 * moveAmount, -2*moveAmount, 10);

    }
}