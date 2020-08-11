package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TwoPlayerTeleOp", group="Zippo")
//@Disabled

//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

public class TwoPlayerTeleOp extends OpMode{

    driveOnlyHardware robot  = new driveOnlyHardware();

    // Create variables for motor power
    private double flPower = 0; //left wheel
    private double frPower = 0; //right wheel
    private double blPower = 0; //front power
    private double brPower = 0; //back power
    private boolean scalePower = false;
    private boolean cornerTurn = true;
    private boolean aCheck = false;
    private boolean driverControl = true;
    private double liftPower = 0; // left lift power
//    private double rlpower = 0; // right lift power



    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.sensorColor.enableLed(false);
        double time =  System.currentTimeMillis();
//        robot.servoLatch.setPosition(-0.1);

    }

    @Override
    public void loop() {
//----------------------------------------------------------------------------------------------------
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MAIN DRIVING CONTROLS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//        When the driver (gamepad1) drives, they drive at full speed, and are able to slow their
//        speed down. The the precision (gamepad2) drives, they go at 30% speed, allowing for more
//        precise control.
//----------------------------------------------------------------------------------------------------


        if ((gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1))
        {
            flPower = gamepad1.left_stick_y * 2;
            brPower = gamepad1.left_stick_y *2;
            frPower = gamepad1.left_stick_y *2;
            blPower = gamepad1.left_stick_y* 2;
            driverControl = true;
        }
        else if ((gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1))
        {
            flPower = gamepad2.left_stick_y * .6;
            brPower = gamepad2.left_stick_y * .6;
            frPower = gamepad2.left_stick_y * .6;
            blPower = gamepad2.left_stick_y* .6;
            driverControl = false;
        }


        else if ((gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_x > .2))
        {
            frPower = gamepad1.left_stick_x * 2;
            blPower = gamepad1.left_stick_x * 2;
            flPower = -gamepad1.left_stick_x * 2;
            brPower = -gamepad1.left_stick_x * 2;
            driverControl = true;
        }
        else if (gamepad2.left_stick_x <-.02 || gamepad2.left_stick_x > .2)
        {
            frPower = gamepad2.left_stick_x * .6;
            blPower = gamepad2.left_stick_x * .6;
            flPower = -gamepad2.left_stick_x * .6;
            brPower = -gamepad2.left_stick_x * .6;
            driverControl = false;
        }
        else
        {
            flPower = 0;
            brPower = 0;
            frPower = 0;
            blPower = 0;
        }

//----------------------------------------------------------------------------------------------------
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>ROTATION<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//        The driver (player 1) can turn the robot using their right stick. By default, they turn
//        from the center, but if they hold the right trigger, they turn from the back corners. When
//        the precision (gamepad2) turns using the bumpers, they turn from the center at 30% speed.
//----------------------------------------------------------------------------------------------------

        if (gamepad2.right_bumper)
        {
            flPower = -.3;
            brPower = .3;
            frPower = .3;
            blPower = -.3;
        }
        else if (gamepad2.left_bumper)
        {
            flPower = .3;
            brPower = -.3;
            frPower = -.3;
            blPower = .3;
        }
        if (gamepad1.right_trigger < .2) {
            if (gamepad1.right_stick_x < -0.1) {
                flPower = 1;
                brPower = -1;
                frPower = -1;
                blPower = 1;
            }
            else if (gamepad1.right_stick_x > 0.1) {
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
            }
            else if (gamepad1.right_stick_x < -0.1) {
                flPower = 1;
                brPower = -1;
                blPower = -1;

            }
        }


//----------------------------------------------------------------------------------------------------
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>OVERCLOCK PROTECTION<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//        Here, we limit the power going to the motors as to not over clock them.
//----------------------------------------------------------------------------------------------------


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

//----------------------------------------------------------------------------------------------------
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>CLAW USAGE<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//        The driver can use the claw by pressing the buttons x or y, one of which opens, the other
//        closes. The precision can use their triggers to open and close the claw.
//----------------------------------------------------------------------------------------------------


        if (gamepad1.x || gamepad2.right_trigger > .4)
        {
            robot.servoClaw.setPosition(1);
        }
        else if (gamepad1.y || gamepad2.left_trigger > .4)
        {
            robot.servoClaw.setPosition(0);
        }

//----------------------------------------------------------------------------------------------------
//      >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>CAPSTONE DROP<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//        By default, both players must do a anime fusion (?) to drop the capstone, both pressing
//        the a button. However, the driver may emergency super duper anime scream overpower
//        auxiliary override this precaution to drop the captsone, by holding a, b, and left trigger.
//        Either way, the claw must be open, allowing the capstone to have a clear drop.
//----------------------------------------------------------------------------------------------------

        if ((gamepad1.a && gamepad2.a) && robot.servoClaw.getPosition()>.5)
        {
            robot.servoDrop.setPosition(1);
        }


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
        if (gamepad1.left_bumper)
        {
            robot.servoTwist.setPower(1);
        }
        else if (gamepad1.right_bumper)
        {
            robot.servoTwist.setPower(-1);
        }
        else
            robot.servoTwist.setPower(0);


        if (gamepad1.dpad_left)
        {
            robot.servoDrag.setPosition(.6);
        }
        else if (gamepad1.dpad_right)
        {
            robot.servoDrag.setPosition(.5);
        }

        if (gamepad1.dpad_up)
        {
            liftPower=.5;
        }
        else if (gamepad1.dpad_down)
        {
            liftPower=-.5;
        }
        else
        {
            liftPower=0;
        }
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
        robot.motorLift.setPower(liftPower);
//        robot.motorLiftR.setPower(rlpower);





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