package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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
    private double liftRPower = 0;
    private double liftLPower = 0;
    private boolean scalePower = false;
    private boolean cornerTurn = true;
    public double drive = 1;
//    private double llpower = 0; // left lift power
//    private double rlpower = 0; // right lift power




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
    public void loop()
    {
        mecanumMove();
        if (gamepad1.dpad_up)
        {
            liftRPower = 1;
            liftLPower = -1;
        }
        else if (gamepad1.dpad_down)
        {
            liftRPower = -1;
            liftLPower = 1;
        }
        else
            {
                liftRPower = 0;
                liftLPower = 0;
            }

        if (gamepad1.x)
        {
            robot.servoGrab.setPosition(0);
        }
        else if(gamepad1.y)
        {
            robot.servoGrab.setPosition(0.9);
        }

        robot.motorLiftR.setPower(liftRPower);
        robot.motorLiftL.setPower(liftLPower);

        //robot.rightLift.setPower(rlpower);



    }

//--------------------------------- FUNCTIONS ----------------------------------------------------

    public void mecanumMove()
    {
        double r = Math.hypot(-gamepad1.left_stick_x,gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) -Math.PI/4;
        double rightX = -gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.motorFrontLeft.setPower(-drive * v1);
        robot.motorFrontRight.setPower(-drive * v2);
        robot.motorBackLeft.setPower(-drive * v3);
        robot.motorBackRight.setPower(-drive * v4);
    }

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