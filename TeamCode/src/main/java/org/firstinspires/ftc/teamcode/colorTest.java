//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//
//@Autonomous(name="colorTest", group="Zippo")
//@Disabled
//public class colorTest extends LinearOpMode {
//    public void runOpMode()
//    {
//
////        BNO055IMU imu;
////
////        TestHardware robot = new TestHardware();
////
////
////
////        robot.init(hardwareMap);
////        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
////        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
////        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
////        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
////        parameters.loggingEnabled = true;
////        parameters.loggingTag = "IMU";
////        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
////        imu = hardwareMap.get(BNO055IMU.class, "imu");
////        imu.initialize(parameters);
////        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
////
////        telemetry.addData("Status", "Resetting Encoders");
////        telemetry.update();
////
////        while (opModeIsActive()) {
////            telemetry.addData("Left Red", robot.sensorColor.red());
////            telemetry.addData("Left Blue", robot.sensorColor.blue());
////            telemetry.addData("Left Green", robot.sensorColor.green());
////            telemetry.addData("Left Red", robot.sensorColor2.red());
////            telemetry.addData("Left Blue", robot.sensorColor2.blue());
////            telemetry.addData("Left Green", robot.sensorColor2.green());
////
////
////            if (robot.sensorColor.red() > robot.sensorColor.blue() + 15 && robot.sensorColor.green() > robot.sensorColor.blue() + 15)
////            {
////                telemetry.addData("Left Block Color", "Yellow");
////            }
////            else
////                telemetry.addData("Left Block Color", "Not Yellow");
////
////            if (robot.sensorColor2.red() > robot.sensorColor2.blue() + 15 && robot.sensorColor2.green() > robot.sensorColor2.blue() + 15)
////            {
////                telemetry.addData("Right Block Color", "Yellow");
////            }
////            else
////                telemetry.addData("Right Block Color", "Not Yellow");
////
////            telemetry.update();
////            sleep(2000);
//        }
//
//
//    }
////}
