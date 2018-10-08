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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class VAPushbot
{
    /* Public OpMode members. */
    public DcMotor  leftbackdrive   = null;
    public DcMotor  rightbackdrive  = null;
    public DcMotor  leftfrontdrive  = null;
    public DcMotor  rightfrontdrive = null;
    public DcMotor  armstringspooler= null;
    public Servo    FrontSpinner   = null;
    public Servo    BackSpinner    = null;
    public Servo    MarkerTipper = null;
    public Servo    ArmClaw  = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public VAPushbot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftbackdrive  = hwMap.get(DcMotor.class, "leftback_drive");
        rightbackdrive = hwMap.get(DcMotor.class, "rightback_drive");
        leftfrontdrive = hwMap.get(DcMotor.class, "leftfront_drive");
        rightfrontdrive= hwMap.get(DcMotor.class, "rightfront_drive");
        armstringspooler= hwMap.get(DcMotor.class, "armstring_spooler");
        leftbackdrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightbackdrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftfrontdrive.setDirection(DcMotor.Direction.FORWARD);
        rightfrontdrive.setDirection(DcMotor.Direction.FORWARD);
        armstringspooler.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftbackdrive.setPower(0);
        rightbackdrive.setPower(0);
        leftfrontdrive.setPower(0);
        rightfrontdrive.setPower(0);
        armstringspooler.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftbackdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftfrontdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        FrontSpinner  = hwMap.get(Servo.class, "Front_Spinner");
        BackSpinner = hwMap.get(Servo.class, "Back_Spinner");
        MarkerTipper = hwMap.get(Servo.class, "Marker_Tipper" );
        ArmClaw  = hwMap.get(Servo.class, "Arm_Claw");
        FrontSpinner.setPosition(MID_SERVO);
        BackSpinner.setPosition(MID_SERVO);
        MarkerTipper.setPosition(MID_SERVO);
        ArmClaw.setPosition(MID_SERVO);
    }

    private class Public {
    }
}

