/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This class defines the hardware of a Servo/Sensor Test Rig.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * REV Smart Robotics Servo not attached to anything: "testServo"
 * REV 2m Distance Sensor: "sensor_range"
 * REV Color Distance Sensor: "sensor_color_distance"
 * Motors as outlined below
 */
public class ServoSensorHardware
{
    /* Public OpMode members. */
    public DcMotor motor1 = null; // front left
    public DcMotor motor2 = null; // front right
    public DcMotor motor3 = null; // back left
    public DcMotor motor4 = null; // back right
    public Servo    testServo   = null;
    public DistanceSensor sensorRange = null;
    public ColorSensor sensorColor = null;
    public DistanceSensor sensorDistance = null;
    public RevTouchSensor digitalTouch = null;

    // Constants
    public static final double MID_SERVO       =  0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public ServoSensorHardware(){ }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        motor1 = hwMap.get(DcMotor.class, "motor1");
        motor2 = hwMap.get(DcMotor.class, "motor2");
        motor3 = hwMap.get(DcMotor.class, "motor3");
        motor4 = hwMap.get(DcMotor.class, "motor4");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setPower(0);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setPower(0);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);
        motor3.setPower(0);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setDirection(DcMotorSimple.Direction.FORWARD);
        motor4.setPower(0);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize installed servos.
        testServo  = hwMap.get(Servo.class, "testServo");
        testServo.setPosition(MID_SERVO);

        // Define and initialize distance sensor as regular
        sensorRange = hwMap.get(DistanceSensor.class, "sensorRange");

        // Define and initialize color/distance sensor
        sensorColor = hwMap.get(ColorSensor.class, "sensorColor");
        sensorDistance = hwMap.get(DistanceSensor.class, "sensorColor");

        // Define and initialize touch sensor
        digitalTouch = hwMap.get(RevTouchSensor.class, "digitalTouch");
    }
 }

