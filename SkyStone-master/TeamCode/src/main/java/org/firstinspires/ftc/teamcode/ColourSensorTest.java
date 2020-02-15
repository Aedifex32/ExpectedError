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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/*
This file contains an autonomous program that you can use to test the hue values of different parts of the field we want to see.
I need you to put the colour sensor over the red bridge line, the blue bridge line, and the grey mats.
Use a camera/phone to record the screen of the driver station phone as you hold the sensor over each piece.
You should only need a maximum of 10 seconds of data for each of the tests, but try not to send me less than 5 per piece.
 */

@Autonomous(name="Colour Sensor Test", group="Pushbot")
public class ColourSensorTest extends LinearOpMode {

    /* Declare OpMode members. */
    DriveTrain driveTrain = new DriveTrain();
    RobotMap robotMap = new RobotMap();
    private ElapsedTime     runtime = new ElapsedTime();
    List<Integer> Red = new ArrayList<>();
    List<Integer> Green = new ArrayList<>();
    List<Integer> Blue = new ArrayList<>();
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

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
        while(opModeIsActive()) {

            Red.add(robotMap.colourSensor.red());
            Green.add(robotMap.colourSensor.green());
            Blue.add(robotMap.colourSensor.blue());


            Color.RGBToHSV((int) (robotMap.colourSensor.red() * SCALE_FACTOR),
                    (int) (robotMap.colourSensor.green() * SCALE_FACTOR),
                    (int) (robotMap.colourSensor.blue() * SCALE_FACTOR),
                    hsvValues);

            List<Integer> RedHues = new ArrayList<Integer>(Red);
            List<Integer> GreenHues = new ArrayList<Integer>(Green);
            List<Integer> BlueHues = new ArrayList<Integer>(Blue);

            telemetry.addData("Red Hues", RedHues);
            telemetry.addData("Green Hues", GreenHues);
            telemetry.addData("Blue Hues", BlueHues);
            telemetry.addData("Total Hue", hsvValues[0]);
            telemetry.update();
            sleep(1000);
        }
    }
}
