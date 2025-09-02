/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcodespain.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodespain.SpainBot2025;

/*
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Auto - LED Stick Testing")
@Config
public class SpainBotLEDStickTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        SpainBot2025 robot = new SpainBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {
            // Test LED Stick by cycling through colors
            telemetry.addLine("Testing LED Stick colors...");
            telemetry.update();
            int[] colors = {
                    0xFF0000, // Red
                    0x00FF00, // Green
                    0x0000FF, // Blue
                    0xFFFF00, // Yellow
                    0xFF00FF, // Magenta
                    0x00FFFF, // Cyan
                    0xFFFFFF  // White
            };
            for (int color : colors) {
                robot.ledStick.setColor(color);
                telemetry.addData("Set LED color to", String.format("#%06X", (0xFFFFFF & color)));
                telemetry.update();
                robot.wait(1); // Hold each color for 1 second
            }

            // Test setting individual LEDs
            telemetry.addLine("Testing individual LED colors...");
            telemetry.update();

            for (int pos = 0; pos <= 9; pos++) {
                int color = colors[pos % colors.length];
                robot.ledStick.setColor(pos, color);
                telemetry.addData(String.format("Set LED %d to color", pos), String.format("#%06X", (0xFFFFFF & color)));
                telemetry.update();
                robot.wait(1); // Hold each LED color for 0.5 seconds
            }

            robot.wait(6);

            robot.ledStick.turnAllOff();


            // stop autonomous and wait for finish
            telemetry.addLine("Testing complete. Stopping autonomous.");
            telemetry.update();
            sleep(30000);
        }
    }   // end runOpMode()
}   // end class
