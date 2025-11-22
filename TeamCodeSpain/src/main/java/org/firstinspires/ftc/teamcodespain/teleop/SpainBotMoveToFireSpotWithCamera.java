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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/*
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Camera Firing Test")
@Config
public class SpainBotMoveToFireSpotWithCamera extends LinearOpMode {

    private static final Pose2d TARGET_POSE = new Pose2d(0, 0, Math.toRadians(135));
    private static final double TARGET_TAG_X = -6.4;
    private static final double TARGET_TAG_Y = 78.5;
    private static final double TARGET_TAG_BEARING = 4.7;
    private static final double DX_TO_FIELD_X = -1.342;
    private static final double DY_TO_FIELD_X = -0.955;
    private static final double DX_TO_FIELD_Y = 1.342;
    private static final double DY_TO_FIELD_Y = 0.955;
    private static final double POSITION_TOLERANCE_IN = 0.5;
    private static final double HEADING_TOLERANCE_DEG = 1.5;

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

                    AprilTagDetection detection = robot.telemetryAprilTag(telemetry);
                    if (detection != null) {
                        TagPoseEstimate estimate = estimatePoseFromDetection(detection);
                        telemetry.addLine(String.format("Estimated Robot Pose: X: %.2f  Y: %.2f  Heading: %.2f deg",
                                estimate.pose.position.x,
                                estimate.pose.position.y,
                                Math.toDegrees(estimate.pose.heading.toDouble())
                        ));
                        telemetry.addLine(String.format("Offset from Target: DX: %.2f  DY: %.2f  Heading Error: %.2f deg",
                                estimate.offsetFromTarget.x,
                                estimate.offsetFromTarget.y,
                                estimate.headingErrorDeg
                        ));

                        double positionError = estimate.offsetFromTarget.norm();
                        double headingError = Math.abs(estimate.headingErrorDeg);

                        if (positionError <= POSITION_TOLERANCE_IN && headingError <= HEADING_TOLERANCE_DEG) {
                            telemetry.addLine("Robot is within tolerance of target position and heading.");
                        } else {
                            telemetry.addLine("Moving robot to target position...");

                            moveRobotToTarget(robot, estimate.pose);
                            telemetry.addLine("Robot moved to target position.");
                        }
                    } else {
                        telemetry.addLine("No AprilTag detected. Cannot estimate pose.");
                    }

                } else if (!yPressed) {
                    telemetry.addLine("Press Y to run AprilTag detection.");
                }

                telemetry.update();
                yPressedLast = yPressed;
                sleep(50);
            }
        }
    }   // end runOpMode()
    private TagPoseEstimate estimatePoseFromDetection(AprilTagDetection detection) {
        double dx = detection.ftcPose.x - TARGET_TAG_X;
        double dy = detection.ftcPose.y - TARGET_TAG_Y;

        double offsetX = DX_TO_FIELD_X * dx + DY_TO_FIELD_X * dy;
        double offsetY = DX_TO_FIELD_Y * dx + DY_TO_FIELD_Y * dy;
        double bearingDelta = detection.ftcPose.bearing - TARGET_TAG_BEARING;

        Pose2d pose = new Pose2d(
                TARGET_POSE.position.x - offsetX,
                TARGET_POSE.position.y - offsetY,
                TARGET_POSE.heading.toDouble() - Math.toRadians(bearingDelta)
        );

        return new TagPoseEstimate(pose, new Vector2d(offsetX, offsetY), bearingDelta);
    }

    private void moveRobotToTarget(SpainBot2025 robot, Pose2d estimatedPose) {
        robot.localizer.setPose(estimatedPose);
        Action moveAction = robot.actionBuilder(estimatedPose)
                .splineToLinearHeading(TARGET_POSE, TARGET_POSE.heading)
                .build();
        Actions.runBlocking(new SequentialAction(moveAction));
        robot.stopMotors();
    }

    private static class TagPoseEstimate {
        final Pose2d pose;
        final Vector2d offsetFromTarget;
        final double headingErrorDeg;

        TagPoseEstimate(Pose2d pose, Vector2d offsetFromTarget, double headingErrorDeg) {
            this.pose = pose;
            this.offsetFromTarget = offsetFromTarget;
            this.headingErrorDeg = headingErrorDeg;
        }
    }
}   // end class


/* AprilTag detection data
Forward Position:
    X: -24
    Y: 24
    Heading: 135
    AprilTag Values:
        XYZ: -0.8, 45.5, 11.5
        PRY: -15.3, -4.4, -12.9
        RBE: 45.5, 1.0, 14.2

Target position:
    X: 0
    Y: 0
    Heading: 135
    AprilTag Values:
        XYZ: -6.4, 78.5, 1.8
        PRY: -15.9, -3.7, -10.2
        RBE: 78.7, 4.7, 1.3

Rear Position:
    X: 24
    Y: -24
    Heading: 135
    AprilTag Values:
        XYZ: -2.9, 98.7, -3.1
        PRY: -14.2, -3.6, -10.6
        RBE: 98.7, 1.7, -1.8

 */