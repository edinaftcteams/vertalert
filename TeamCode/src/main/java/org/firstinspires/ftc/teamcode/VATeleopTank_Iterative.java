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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: Teleop Tank", group="Pushbot")

public class VATeleopTank_Iterative extends OpMode{

    VAPushbot robot       = new VAPushbot();

    @Override
    public void init() {

        robot.init(hardwareMap);

        telemetry.addData("Say", "Catch Tha Dub Bois");    //
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        double drive;
        double rotate1;
        double rotate2;

        drive = -gamepad1.left_stick_y;


        robot.leftbackdrive.setPower(drive);
        robot.rightbackdrive.setPower(drive);
        robot.rightfrontdrive.setPower(drive);
        robot.leftfrontdrive.setPower(drive);

        rotate1 = gamepad1.right_stick_x;
        rotate2 = -gamepad1.right_stick_x;

        robot.leftbackdrive.setPower(rotate1);
        robot.rightbackdrive.setPower(rotate2);
        robot.leftfrontdrive.setPower(rotate1);
        robot.rightfrontdrive.setPower(rotate2);


        if (gamepad1.y) {
            robot.Sweeper.setPower(VAPushbot.ARM_UP_POWER);
        } else if (gamepad1.a) {
            robot.Sweeper.setPower(VAPushbot.ARM_DOWN_POWER);
        } else {
            robot.Sweeper.setPower(0.0);
        }

        if (gamepad1.dpad_up) {
            robot.armlifter.setPower(VAPushbot.ARM_UP_POWER);
        } else if (gamepad1.dpad_down) {
            robot.armlifter.setPower(VAPushbot.ARM_DOWN_POWER);
        } else {
            robot.armlifter.setPower(0.0);
        }

        if (gamepad1.left_bumper) {
            robot.MarkerTipper.setPosition(1.0);
        } else if (gamepad1.right_bumper) {
            robot.MarkerTipper.setPosition(0.0);
        }

            telemetry.addData("drive", "%.2f", drive);
            telemetry.addData("rotate1", "%.2f", rotate1);
            telemetry.addData("rotate2", "%.2f", rotate2);
}



        @Override
        public void stop () {
        }
    }