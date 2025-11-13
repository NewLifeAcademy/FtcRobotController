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

package org.firstinspires.ftc.teamcodespain.teleop;

import static org.firstinspires.ftc.teamcode.SpainBotBlockCompanion.robotMotorPower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Position;
import org.firstinspires.ftc.teamcodespain.SpainBot2025;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Camera Firing Test")
@Config
public class SpainBotMoveToFireSpotWithCamera extends LinearOpMode {

    @Override
    public void runOpMode() {
        SpainBot2025 robot = new SpainBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.initAprilTagDetection();
        waitForStart();

        if (opModeIsActive()) {

            boolean yPressedLast = false;

            while (opModeIsActive()) {
                // Use Robot Mote Power function from Block Companion to drive robot
//                robotMotorPower(
//                        -gamepad1.left_stick_y,
//                        -gamepad1.left_stick_x,
//                        -gamepad1.right_stick_x
//                );

                robot.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                // Use D-Pad Up/Down to start/stop camera streaming
                if (gamepad1.dpad_down) {
                    robot.visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    robot.visionPortal.resumeStreaming();
                }
                // Check for Y button press to run AprilTag detection
                boolean yPressed = gamepad1.y;

                if (yPressed && !yPressedLast) {
                    telemetry.addLine("Running AprilTag detection...");
                    telemetry.update();

                    robot.telemetryAprilTag(telemetry);

                    telemetry.addLine("Detection complete. Press Y to run again.");
                } else if (!yPressed) {
                    telemetry.addLine("Press Y to run AprilTag detection.");
                }

                telemetry.update();
                yPressedLast = yPressed;
                sleep(50);
            }
        }
    }   // end runOpMode()
}   // end class


/* AprilTag detection data
Target position:
    X: 0
    Y: 0
    Heading: 135
    AprilTag Values:
        XYZ: -6.4, 78.5, 1.8
        PRY: -15.9, -3.7, -10.2
        RBE: 78.7, 4.7, 1.3

Forward Position:
    X: -24
    Y: 24
    Heading: 135
    AprilTag Values:
        XYZ: -2.9, 98.7, -3.1
        PRY: -14.2, -3.6, -10.6
        RBE: 98.7, 1.7, -1.8

Rear Position:
    X: 24
    Y: -24
    Heading: 135
    AprilTag Values:
        XYZ: -0.8, 45.5, 11.5
        PRY: -15.3, -4.4, -12.9
        RBE: 45.5, 1.0, 14.2
 */