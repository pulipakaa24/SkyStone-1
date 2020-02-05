package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static android.os.SystemClock.sleep;

@TeleOp(name="MecanumTeleOp", group="Zippo")
//@Disabled

//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

public class mecanumTeleOp extends OpMode{

    mecanumHardware robot  = new mecanumHardware();

    // Create variables for motor power
    private double flPower = 0; //left wheel
    private double frPower = 0; //right wheel
    private double blPower = 0; //front power
    private double brPower = 0; //back power
    private boolean scalePower = false;
    private boolean cornerTurn = true;
    private boolean aCheck = false;
    private double llpower = 0; // left lift power
    private double rlpower = 0; // right lift power
    private double dragPower = .9;



    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.sensorColor.enableLed(false);
        double time =  System.currentTimeMillis();
        robot.servoLatch.setPosition(-0.1);

    }

    @Override
    public void loop() {
//      MAIN DRIVING CONTROLS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//        lPower = gamepad1.right_stick_y;
//        rPower = gamepad1.right_stick_y;
//        fPower = -gamepad1.right_stick_x;
//        bPower = -gamepad1.right_stick_x;
        if ((gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1)||(gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1))
        {
            flPower = gamepad1.left_stick_y * 2;
            brPower = gamepad1.left_stick_y *2;
            frPower = gamepad1.left_stick_y *2;
            blPower = gamepad1.left_stick_y* 2;
        }


        else if ((gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_x > .2) || (gamepad2.left_stick_x <-.02 || gamepad2.left_stick_x > .2))
        {
            frPower = gamepad1.left_stick_x * 2;
            blPower = gamepad1.left_stick_x * 2;
            flPower = -gamepad1.left_stick_x * 2;
            brPower = -gamepad1.left_stick_x * 2;
        }
        else
        {
            flPower = 0;
            brPower = 0;
            frPower = 0;
            blPower = 0;
        }

//        if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1)
//        {
//            flPower = gamepad1.left_stick_y * 2;
//            brPower = gamepad1.left_stick_y *2;
//            frPower = gamepad1.left_stick_y *2;
//            blPower = gamepad1.left_stick_y* 2;
//        }
//
//
//        else if (gamepad2.left_stick_x < -0.2 || gamepad2.left_stick_x > .2)
//        {
//            frPower = gamepad1.left_stick_x * 2;
//            blPower = gamepad1.left_stick_x * 2;
//            flPower = -gamepad1.left_stick_x * 2;
//            brPower = -gamepad1.left_stick_x * 2;
//        }
//        else
//        {
//            flPower = 0;
//            brPower = 0;
//            frPower = 0;
//            blPower = 0;
//        }

        //rotate

        if (true) {
            if (gamepad1.right_stick_x < -0.1 || gamepad2.right_stick_x < -0.1) {
                flPower = 1;
                brPower = -1;
                frPower = -1;
                blPower = 1;
            } else if (gamepad1.right_stick_x > 0.1 || gamepad2.right_stick_x> 0.1) {
                flPower = -1;
                brPower = 1;
                frPower = 1;
                blPower = -1;
            }
        }
        else
        {
            if (gamepad1.right_stick_x > 0.1) {
                flPower = -1;

                frPower = -1;
                blPower = 1;
            } else if (gamepad1.right_stick_x < -0.1) {
                flPower = 1;
                brPower = -1;
                blPower = -1;

            }
        }

        if (gamepad1.dpad_up || gamepad2.dpad_up)
        {
            llpower = .8;
            rlpower = .8;
        }
        else if (gamepad1.dpad_down || gamepad2.dpad_down)
        {
            llpower = -.8;
            rlpower = -.8;

        }

        else
        {
            llpower = 0.001;
            rlpower = 0.001;
        }

        if (gamepad1.x || gamepad2.x)
        {

            robot.servoClaw.setPower(.5);

        }
        else if(gamepad1.y || gamepad2.y)
        {
            robot.servoClaw.setPower(-.5);
//            double position = 0.49;
//            robot.servoClaw.setPosition(position);
//            while (position >=0)
//            {
//                position -= 0.1;
//                robot.servoClaw.setPosition(position);
//                sleep(100);
//            }
        }

        if(gamepad2.a)
        {
            robot.servoLatch.setPosition(0);
        }
        else if(gamepad2.b)
        {
            robot.servoLatch.setPosition(1);
        }

        if (gamepad1.a)
        {
            robot.servoDrag2.setPosition(0);
            robot.servoDrag1.setPosition(0);
        }
        else if (gamepad1.b)
        {
            robot.servoDrag2.setPosition(.9);
            robot.servoDrag1.setPosition(.9);
        }
        if(gamepad1.right_bumper || gamepad2.right_bumper)
        {
            robot.servoTwist.setPosition(0);
        }
        else if(gamepad1.left_bumper || gamepad2.left_bumper)
        {
            robot.servoTwist.setPosition(.5);
        }




        if(frPower > 1)
        {
            frPower = 1;
        }
        else if (frPower < -1)
        {
            frPower = -1;
        }

        if(flPower > 1)
        {
            flPower = 1;
        }
        else if (flPower < -1)
        {
            flPower = -1;
        }

        if(blPower > 1)
        {
            blPower = 1;
        }
        else if (blPower < -1)
        {
            blPower = -1;
        }
        if(brPower > 1)
        {
            brPower = 1;
        }
        else if (brPower < -1)
        {
            brPower = -1;
        }
//
        if (gamepad1.left_trigger > .1)
        {
            scalePower = true;
        }
        else
        {
            scalePower = false;
        }
        if (gamepad1.right_trigger > .1)
        {
            cornerTurn = true;
        }
        else
        {
            cornerTurn = false;
        }
        //BringDown hook = new BringDown();


//      SCALING POWERS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        if(scalePower)
        {
            flPower *= .3;
            brPower *= .3;
            frPower *= .3;
            blPower *= .3;
        }

//      SETTING POWERS AND POSITIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        robot.motorFrontLeft.setPower(flPower);
        robot.motorBackRight.setPower(brPower);
        robot.motorFrontRight.setPower(frPower);
        robot.motorBackLeft.setPower(blPower);
        robot.motorLiftL.setPower(llpower);
        robot.motorLiftR.setPower(rlpower);





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