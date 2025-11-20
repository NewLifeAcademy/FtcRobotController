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

package org.firstinspires.ftc.teamcodeiron.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Position;
import org.firstinspires.ftc.teamcodeiron.IronBot2025;

/*
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Auto - Firing Waypoint Test")
@Config
public class IronBotFiringWaypointTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        IronBot2025 robot = new IronBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            Pose2d startPose = new Pose2d(-60, 8, Math.toRadians(90));
            Position firePosition = new Position(new Pose2d(-36, 32, Math.toRadians(135)), Math.toRadians(135));
            double fireTime = 1.5;

            robot.localizer.setPose(startPose);
            Action action = robot.actionBuilder(startPose)
                    // Move to fire position
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(firePosition.getPose(), firePosition.getTangent())
                    // fire preloaded
                    .waitSeconds(fireTime)
                    // Move to spike 3 (PPG)
                    .setTangent(Math.toRadians(45))
                    .splineToLinearHeading(new Pose2d(-11.5, 32, Math.toRadians(90)), Math.toRadians(90))
                    // intake PPG
                    .lineToY(44)
                    // Move to fire position
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(firePosition.getPose(), firePosition.getTangent())
                    // fire PPG
                    .waitSeconds(fireTime)
                    // Move to spike 2 (PGP)
                    .setTangent(Math.toRadians(135))
                    .splineToLinearHeading(new Pose2d(11.5, 32, Math.toRadians(90)), Math.toRadians(90))
                    // intake PGP
                    .lineToY(44)
                    // Move to fire position
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(firePosition.getPose(), firePosition.getTangent())
                    // fire PGP
                    .waitSeconds(fireTime)
                    // Move to spike 1 (GPP)
                    .setTangent(Math.toRadians(135))
                    .splineToLinearHeading(new Pose2d(34.5, 32, Math.toRadians(90)), Math.toRadians(90))
                    // intake GPP
                    .lineToY(44)
                    // Move to fire position
                    .setTangent(Math.toRadians(90))
                    .splineToLinearHeading(firePosition.getPose(), firePosition.getTangent())
                    // fire GPP
                    .waitSeconds(fireTime)
                    // Move to park
                    .setTangent(Math.toRadians(135))
                    .splineToLinearHeading(new Pose2d(-52, 8, Math.toRadians(0)), Math.toRadians(0))
                    // end autonomous
                    .build();

            Actions.runBlocking(new SequentialAction(action));

            // AprilTag detection test
            robot.initAprilTagDetection();
            robot.telemetryAprilTag(telemetry);

            // stop autonomous and wait for finish
            telemetry.addLine("Testing complete. Stopping autonomous.");
            telemetry.update();
            sleep(30000);
        }
    }   // end runOpMode()
}   // end class
