package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestTeleop", group="Zippo")
//@Disabled

//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

public class TestTeleop extends OpMode{

    TestHardware robot  = new TestHardware();

    // Create variables for motor power
    private double lPower = 0; //left wheel
    private double rPower = 0; //right wheel
    private double fPower = 0; //front power
    private double bPower = 0; //back power
    private double llpower = 0; // left lift power
    private double rlpower = 0; // right lift power
    private double stpower = 0;//servo twist
    private double sgpower = 0;//servo grab
    private double sdpower = 0;//servo drag





    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.sensorColor.enableLed(false);
        double time =  System.currentTimeMillis();

    }

    @Override
    public void loop() {

//      MAIN DRIVING CONTROLS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//        lPower = gamepad1.right_stick_y;
//        rPower = gamepad1.right_stick_y;
//        fPower = -gamepad1.right_stick_x;
//        bPower = -gamepad1.right_stick_x;
        if (gamepad1.left_stick_y != 0)
        {
            fPower = gamepad1.left_stick_y;
            bPower = gamepad1.left_stick_y;
        }
        else if (gamepad2.left_stick_x != 0)
        {
            fPower = gamepad2.left_stick_x;
            bPower = gamepad2.left_stick_x;
        }

        else
        {
            fPower = 0;
            bPower = 0;
        }
        if (gamepad1.left_stick_x != 0)
        {
            rPower = gamepad1.left_stick_x;
            lPower = gamepad1.left_stick_x;
        }
        else if (gamepad2.left_stick_x != 0)
        {
            rPower = gamepad2.left_stick_x;
            lPower = gamepad2.left_stick_x;
        }


        else
        {
            rPower = 0;
            lPower = 0;
        }

        //rotate
        if(gamepad1.right_stick_x > 0 || gamepad2.right_stick_x > 0)
        {
            lPower = -.2; //lpower used to be 1
            rPower = .2; //rpower used to be -1
            fPower = -.2;
            bPower = .2;
        }
        else if(gamepad1.right_stick_x < 0 || gamepad2.right_stick_x < 0)
        {
            lPower = .2; //lpower used to be -1
            rPower = -.2; //used to be 1
            fPower = .2;
            bPower = -.2;
        }
        if (gamepad1.dpad_up || gamepad2.dpad_up)
        {
            llpower = .5;
            rlpower = .5;
        }
        else if (gamepad1.dpad_down || gamepad2.dpad_down)
        {
            llpower = -.5;
            rlpower = -.5;

        }
        else
        {
            llpower = 0;
            rlpower = 0;


        }

        if (gamepad1.left_bumper ||gamepad2.left_bumper)
        {
            stpower = .3;
        }
        else if(gamepad1.right_bumper||gamepad2.right_bumper)
        {
            stpower = -.3;
        }
        else
        {
            stpower = 0;
        }
        if (gamepad1.a ||gamepad2.a)
        {
            sgpower = .3;
        }
        else if(gamepad1.b||gamepad2.b)
        {
            sgpower = -.3;
        }
        else
        {
            sgpower = 0;
        }
        if (gamepad1.x ||gamepad2.x)
        {
            sdpower = .5;
        }
        else if(gamepad1.y||gamepad2.y)
        {
            sdpower = -.5;
        }
        else
        {
            sdpower = 0;
        }




        if(rPower > 1)
        {
            rPower = 1;
        }
        else if (rPower < -1)
        {
            rPower = -1;
        }

        if(fPower > 1)
        {
            fPower = 1;
        }
        else if (fPower < -1)
        {
            fPower = -1;
        }

        if(bPower > 1)
        {
            bPower = 1;
        }
        else if (bPower < -1)
        {
            bPower = -1;
        }

        //BringDown hook = new BringDown();


//      SCALING POWERS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        if(gamepad1.right_bumper)
        {
            lPower *= 0.3;
            rPower *= 0.3;
            fPower *= 0.3;
            bPower *= 0.3;
        }

//      SETTING POWERS AND POSITIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        robot.motorLeft.setPower(fPower);
        robot.motorRight.setPower(bPower);
        robot.motorFront.setPower(rPower);
        robot.motorBack.setPower(lPower);
        robot.leftLift.setPower(llpower);
        robot.rightLift.setPower(rlpower);
        robot.servoDrag.setPower(sdpower);
        robot.servoGrab.setPower(sgpower);
        robot.servoTwist.setPower(stpower);




//      TELEMETRY
//        telemetry.addData("Left Motor Power", gamepad1.left_stick_y);
//        telemetry.addData("Right Motor Power", gamepad1.left_stick_y);
//        telemetry.addData("Front Motor Power", gamepad1.left_stick_x);
//        telemetry.addData("Back Motor Power", gamepad1.left_stick_x);
//        telemetry.addData("Hook Motor Power", gamepad2.left_stick_y);
//        telemetry.addData("Arm Tilt Motor Power", aPowerTIlt);
//        telemetry.addData("Arm Ext Motor Power", aPowerExt);
//        telemetry.addData("Box intake Power", boxPower);
//        telemetry.addData("Right Distance", robot.sensorRangeR.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Left Distance", robot.sensorRangeL.getDistance(DistanceUnit.INCH));
    }

//--------------------------------- FUNCTIONS ----------------------------------------------------

    public double scalePower(double power1)
    {
        double power2;
        if(power1 > 0){
            power2 = Math.pow(Math.abs(power1), 0.6);
        } else if(power1 < 0){
            power2 = -Math.pow(Math.abs(power1), 0.6);
        } else{
            power2 = 0;
        }

        return power2;
    }
}