package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel: front right drive motor: m0
 * Motor channel: front left drive motor: m1
 * Motor channel: rear right drive motor: m2
 * Motor channel: rear left drive motor: m3
 *
 */
public class DriveModuleHardware
{
    /* Public OpMode members. Motors are numbered from front to back according to simple diagram for gear ratios.png
    * m0 = FR
    * m1 = FL
    * m2 = RR
    * m3 = RL*/
    public DcMotor m0 = null, m1 = null, m2 = null, m3 = null;
    // Initialize pointers to motors in an ArrayList
    public ArrayList<DcMotor> motors = new ArrayList<>();

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public DriveModuleHardware() { }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        motors.add(m0);
        motors.add(m1);
        motors.add(m2);
        motors.add(m3);

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Iterate through ArrayList and intialize each motor
        for (DcMotor m : motors) {
            m = hwMap.get(DcMotor.class, "m"+motors.indexOf(m));
            if (motors.indexOf(m) % 2 == 0) {
                m.setDirection(DcMotor.Direction.FORWARD);
            } else {
                m.setDirection(DcMotor.Direction.REVERSE);
            }
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        }
    }
 }

