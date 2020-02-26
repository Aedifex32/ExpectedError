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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Timer;
import java.util.concurrent.TimeUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to
 * the Driver Station OpMode list
 */

@TeleOp(name="MecanumDriveRevamp", group="Linear Opmode")

public class MecanumDriveRevamp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timer = new ElapsedTime();
    DriveTrain driveTrain = new DriveTrain();
    RobotMap robotMap = new RobotMap();
    double direction_y;
    double direction_x;
    double rotation;
    double DEADZONE = 0.3;
    double magnitude;
    double angle;
    double invertDrive = 1;
    double percentSpeed = 1;
    boolean isFirst = true;
    double speedPID = 0.0005;

    @Override
    public void runOpMode() {
        driveTrain.init(hardwareMap);
        robotMap.init(hardwareMap);

        invertDrive = 1;
        percentSpeed = 1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            getInput();
            doDead();
            setSpeed();
            toPolar();
            if (gamepad1.x) {
                doInvert();
            }
            DriveTrain.drivePolar(magnitude, angle, rotation, invertDrive, percentSpeed);

            if (gamepad2.x) {
                robotMap.leftClaw.setPosition(0.4);
                robotMap.rightClaw.setPosition(0.6);
            } else {
                robotMap.leftClaw.setPosition(1);
                robotMap.rightClaw.setPosition(0);
            }

            if (gamepad2.right_bumper) {
                robotMap.rightSlide.setPower(0.75);
            } else {
                robotMap.rightSlide.setPower(0);
            }

            if (gamepad2.left_bumper) {
                robotMap.rightSlide.setPower(-0.75);
            } else {
                robotMap.rightSlide.setPower(0);
            }

            if (gamepad2.dpad_down) {
                robotMap.hookLeft.setPosition(0.6);
                robotMap.hookRight.setPosition(0.5);
            } else {
                robotMap.hookLeft.setPosition(0.9);
                robotMap.hookRight.setPosition(0);
            }

            if (gamepad2.a) {
                robotMap.pinchRight.setPosition(0);
            } else {
                robotMap.pinchRight.setPosition(0.9);
            }

            if (gamepad2.y) {
                robotMap.rightGate.setPosition(0.5);
                robotMap.leftGate.setPosition(0.4);

            } else {
                robotMap.rightGate.setPosition(1);
                robotMap.leftGate.setPosition(0);

            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", DriveTrain.FLPower, DriveTrain.FRPower, DriveTrain.BLPower, DriveTrain.BRPower);
            telemetry.update();
        }
}

    //Start of Drive Setup
    protected void doDead() {

        if(Math.abs(gamepad1.left_stick_x) < DEADZONE){
            direction_x = 0;
        }
        if(Math.abs(-gamepad1.left_stick_y) < DEADZONE){
            direction_y = 0;
        }
        if(Math.abs(gamepad1.right_stick_x) < DEADZONE){
            rotation = 0;
        }
        if((Math.abs(-gamepad1.left_stick_y)) < DEADZONE && (Math.abs(gamepad1.left_stick_x)) < DEADZONE){
            isFirst = true;
        }
    }

    protected void getInput() {
        if((Math.abs(-gamepad1.left_stick_y)) >= DEADZONE && isFirst){
            timer.reset();
            isFirst = false;
        }
        if(timer.time() <= 2){
            direction_y = (-gamepad1.left_stick_y * speedPID * timer.time(TimeUnit.MILLISECONDS));
            direction_x = (gamepad1.left_stick_x * speedPID * timer.time(TimeUnit.MILLISECONDS));
        }
        else {
            direction_y = -gamepad1.left_stick_y;
            direction_x = gamepad1.left_stick_x;
        }
        rotation = gamepad1.right_stick_x;
    }

    protected void toPolar() {
        magnitude = Math.sqrt((direction_x * direction_x) + (direction_y * direction_y)); // sqrt(X^2 + Y^2)
        if (magnitude > 1.0) {  // If Magnitude over 1 set it to 1
            magnitude = 1.0;
        }
        angle = Math.atan2(direction_x , direction_y ) * (180 / Math.PI);  // arctan(y/x) Calculates the angle of the y and x point then converts to radians
    }

    protected void doInvert(){
        invertDrive = invertDrive * -1;
    }

    protected void setSpeed(){
        if(gamepad1.dpad_down){
            percentSpeed = 0.25;
        }
        else if(gamepad1.dpad_left){
            percentSpeed = 0.5;
        }
        else if(gamepad1.dpad_up){
            percentSpeed = 1;
        }
    }

    }
    //End of Drive Setup