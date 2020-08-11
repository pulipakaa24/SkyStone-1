package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Servo Scan", group = "Servo")

public class ServoTestProgram extends LinearOpMode {

    /* Make sure servo is set to angular mode! */

    static final double MIN_POS = 0.0;
    static final double MAX_POS = 1.0;
    static final double INCREMENT   = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   0;     // period of each cycle - if INCREMENT is sufficiently small CYCLE_MS can be set to 0 because of execution time
    static final double CENTER_POS = (MIN_POS+MAX_POS)/2;

    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position

    ServoSensorHardware      robot   = new ServoSensorHardware();
    ElapsedTime     runtime = new ElapsedTime();

    public boolean closeEnough(double val1, double val2) {
        return Math.abs(val1-val2) < 2*INCREMENT;
    }

    public void servoMove(double targetPosition, double timeout) {
        // Setup
        double ElapsedTimeAtStart = runtime.milliseconds();
        boolean finished = false;

        // Check if opMode is active
        while(opModeIsActive()){
            // Check if servo is at target position
            if (closeEnough(robot.testServo.getPosition(), targetPosition)) {
                position = targetPosition;
                telemetry.addLine("Servo already at target position");
                telemetry.update();
                finished = true;
            }
            // Check if servo's position is less than the target position and a previous condition hasn't been met
            else {
                if (robot.testServo.getPosition() - targetPosition > 0 ) {
                    position -= INCREMENT;
                }
                else {
                    position += INCREMENT;
                }
            }
            if (closeEnough(position, targetPosition)) {
                finished = true;
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.3f", position);
            telemetry.update();

            // Set the servo to the new position and pause;
            robot.testServo.setPosition(position);
            sleep(CYCLE_MS);
            idle();

            // Check if timed out
            if(runtime.milliseconds() - ElapsedTimeAtStart >= timeout) {
                telemetry.addLine("Servo move timed out");
                telemetry.update();
                finished = true;
            }

            // If finished with the move, exit the method
            if(finished) {
                break;
            }
        }
    }

    public void servoMove(double target) {
        servoMove(target, 60000);
    }

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.testServo.setPosition(CENTER_POS);

        telemetry.addLine("Press start to run the program.");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            // Game code goes here
        }

    }

}
