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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Red Foundation (Old)", group="Pushbot")
@Disabled
public class RedFoundationAuto extends LinearOpMode {

    /* Declare OpMode members. */
    DriveTrain driveTrain = new DriveTrain();
    RobotMap robotMap = new RobotMap();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.75;
    static final double     BACKWARD_SPEED = -0.75;
    static final double     TURN_SPEED    = 0.6;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        driveTrain.init(hardwareMap);
        robotMap.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //drive to foundation
        driveTrain.FRMotor.setPower(BACKWARD_SPEED);
        driveTrain.BRMotor.setPower(BACKWARD_SPEED);
        driveTrain.FLMotor.setPower(BACKWARD_SPEED);
        driveTrain.BLMotor.setPower(BACKWARD_SPEED);
        sleep(3000);
        //stop motion
        driveTrain.FRMotor.setPower(0);
        driveTrain.BRMotor.setPower(0);
        driveTrain.FLMotor.setPower(0);
        driveTrain.BLMotor.setPower(0);
        sleep(500);
        //hook onto foundation
        robotMap.rightGate.setPosition(0.5);
        robotMap.leftGate.setPosition(0.4);
        sleep(1000);
        //move towards wall
        driveTrain.FRMotor.setPower(FORWARD_SPEED);
        driveTrain.BRMotor.setPower(FORWARD_SPEED);
        driveTrain.FLMotor.setPower(FORWARD_SPEED);
        driveTrain.BLMotor.setPower(FORWARD_SPEED);
        sleep(2000);
        //stop right side motion to start to turn
        driveTrain.FRMotor.setPower(0);
        driveTrain.BRMotor.setPower(0);
        sleep(2000);
        //stop motion
        driveTrain.FLMotor.setPower(0);
        driveTrain.BLMotor.setPower(0);
        sleep(500);
        //release foundation
        robotMap.rightGate.setPosition(1);
        robotMap.leftGate.setPosition(0);
        sleep(1000);
        //move away from the foundation an inch or so
        driveTrain.FRMotor.setPower(FORWARD_SPEED);
        driveTrain.BRMotor.setPower(FORWARD_SPEED);
        driveTrain.FLMotor.setPower(FORWARD_SPEED);
        driveTrain.BLMotor.setPower(FORWARD_SPEED);
        sleep(500);
        //stop motion
        driveTrain.FRMotor.setPower(0);
        driveTrain.BRMotor.setPower(0);
        driveTrain.FLMotor.setPower(0);
        driveTrain.BLMotor.setPower(0);
        //put gates back down
        robotMap.rightGate.setPosition(0.5);
        robotMap.leftGate.setPosition(0.4);
        sleep(1000);
        //push foundation to corner
        driveTrain.FRMotor.setPower(BACKWARD_SPEED);
        driveTrain.BRMotor.setPower(BACKWARD_SPEED);
        driveTrain.FLMotor.setPower(BACKWARD_SPEED);
        driveTrain.BLMotor.setPower(BACKWARD_SPEED);
        sleep(2500);
        //stop motion
        driveTrain.FRMotor.setPower(0);
        driveTrain.BRMotor.setPower(0);
        driveTrain.FLMotor.setPower(0);
        driveTrain.BLMotor.setPower(0);
        sleep(500);
        //move away from foundation
        driveTrain.FRMotor.setPower(FORWARD_SPEED);
        driveTrain.BRMotor.setPower(FORWARD_SPEED);
        driveTrain.FLMotor.setPower(FORWARD_SPEED);
        driveTrain.BLMotor.setPower(FORWARD_SPEED);
        sleep(500);
        //stop motion
        driveTrain.FRMotor.setPower(0);
        driveTrain.BRMotor.setPower(0);
        driveTrain.FLMotor.setPower(0);
        driveTrain.BLMotor.setPower(0);
        sleep(500);
        //strafe towards wall
        double magnitude = 1;
        double angle = Math.atan2(-1 , 0) * (180 / Math.PI);
        double rotation = 0;
        double invertDrive = 1;
        double percentSpeed = 0.75;
        DriveTrain.drivePolar(magnitude, angle, rotation, invertDrive, percentSpeed);
        sleep(2000);
        //stop motion
        driveTrain.FRMotor.setPower(0);
        driveTrain.BRMotor.setPower(0);
        driveTrain.FLMotor.setPower(0);
        driveTrain.BLMotor.setPower(0);
        sleep(500);
        //drive to under bridge
        driveTrain.FRMotor.setPower(FORWARD_SPEED);
        driveTrain.BRMotor.setPower(FORWARD_SPEED);
        driveTrain.FLMotor.setPower(FORWARD_SPEED);
        driveTrain.BLMotor.setPower(FORWARD_SPEED);
        sleep(2000);
        //stop motion and end autonomous
        driveTrain.FRMotor.setPower(0);
        driveTrain.BRMotor.setPower(0);
        driveTrain.FLMotor.setPower(0);
        driveTrain.BLMotor.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
