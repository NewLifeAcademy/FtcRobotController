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

package org.firstinspires.ftc.teamcodealpha.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcodealpha.AlphaBot2024;
import org.firstinspires.ftc.teamcodealpha.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Auto - Specimens", preselectTeleOp = "IntoTheDeep")

@Config
public class AlphaPlanRightSpecimens extends LinearOpMode {
    public static double LIFT_ASCENT_SPEED = 1;
    public static double LIFT_DESCENT_SPEED = 1;
    public static double TILT_SPEED = 1;

    public static double START_POS_X = -10;
    public static double START_POS_Y = 62;
    public static double START_POS_HEADING = 270;
    public static double SAMPLE_AREA_X = -52;
    public static double SAMPLE_AREA_Y = 48;
    public static double SAMPLE_AREA_HEADING = 270;
    public static int SAMPLE_LIFT_HEIGHT = 0;
    public static double SAMPLE_ONE_X = -52;
    public static double SAMPLE_ONE_Y = 36;
    public static double SAMPLE_ONE_HEADING = 270;

    public static double SAMPLE_OBS_DROP_X = -48;
    public static double SAMPLE_OBS_DROP_Y = 52;
    public static double SAMPLE_OBS_DROP_HEADING = 90;

    public static double SAMPLE_OBS_WAIT_X = -48;
    public static double SAMPLE_OBS_WAIT_Y = 46;
    public static double SAMPLE_OBS_WAIT_HEADING = 90;
    private static final boolean USE_APRILTAG_CORRECTIONS = false;  // true for to use webcam and april tag detection for micro adjustments

    private static final double APRILTAG_TARGET_X = -3.5;  // Camera is 3.5 inches to the right of the robot center
    private static final double APRILTAG_TARGET_Y = 16.35;

    private static final double APRILTAG_TARGET_HEADING = 0;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    // Create all waypoints
    private Pose2d startPose = new Pose2d(START_POS_X, START_POS_Y, Math.toRadians(START_POS_HEADING));
    private Pose2d sampleArea = new Pose2d(SAMPLE_AREA_X, SAMPLE_AREA_Y, Math.toRadians(SAMPLE_AREA_HEADING));
    private Pose2d sampleOne = new Pose2d(SAMPLE_ONE_X, SAMPLE_ONE_Y, Math.toRadians(SAMPLE_ONE_HEADING));

    private Pose2d sampleObsDrop = new Pose2d(SAMPLE_OBS_DROP_X, SAMPLE_OBS_DROP_Y, Math.toRadians(SAMPLE_OBS_DROP_HEADING));

    private Pose2d sampleObsWait = new Pose2d(SAMPLE_OBS_WAIT_X, SAMPLE_OBS_WAIT_Y, Math.toRadians(SAMPLE_OBS_WAIT_HEADING));


    @Override
    public void runOpMode() {
        AlphaBot2024 drive = new AlphaBot2024(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        initAprilTag();

        /* Post Jan 7th 2024 updates
        // Set tilt motor to brake - helps with fastened the specimen by keeping the tilt motor from moving
        drive.LiftTiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */
        drive.lowerOdometryWheel();

        waitForStart();

        if (opModeIsActive()) {
            drive.resetLiftEncoders();
            drive.resetTiltEncoder();
            drive.closeClawAndWait();
            drive.retractClawArm();

//            drive.openClaw();
//            sleep(2000);
//            drive.clawLevelNeutral();
//            drive.closeClaw();
//            sleep(2000);
//            drive.clawLevelDown();
//            sleep(2000);
//            drive.clawLevelNeutral();
//            sleep(2000);
//            drive.clawLevelUp();
//            sleep(2000);
//            drive.clawLevelNeutral();
//            drive.openClaw();
//            sleep(30000);


            // A.1 - Define the starting point
            drive.setPoseEstimate(startPose);

            // A.2 - Drive to sample area
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(sampleArea)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            //open claw
            drive.openClaw();

            // lower claw level
            drive.clawLevelUp();

            // A.3 - Use AprilTag for heading correction
            telemetryAprilTag(telemetry);
            telemetry.update();

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            //if there is one and only one object in current detections
            if (USE_APRILTAG_CORRECTIONS && currentDetections.size() == 1) {
                //get the first detection
                AprilTagDetection detection = currentDetections.get(0);
                // get yaw value from detection
                double yaw = detection.ftcPose.yaw;
                // get x value from detection
                double x = detection.ftcPose.x;
                // get y value from detection
                double y = detection.ftcPose.y;

                //set values of sampleOne based on detection
                Pose2d sampleOneCorrection = new Pose2d(
                        sampleOne.getX()-(APRILTAG_TARGET_Y-y),
                        sampleOne.getY()-(APRILTAG_TARGET_X-x),
                        //basketApproach.getHeading() - (APRILTAG_TARGET_HEADING + Math.toRadians(yaw)));
                        sampleOne.getHeading());

                // write april tag target to telemetry
                telemetry.addLine(String.format("April Tag Target: %6.1f %6.1f %6.1f", APRILTAG_TARGET_X, APRILTAG_TARGET_Y, APRILTAG_TARGET_HEADING));
                // write detection to telemetry
                telemetry.addLine(String.format("April Tag Detection: %6.1f %6.1f %6.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw));
                // write sampleOneCorrection to telemetry
                telemetry.addLine(String.format("Sample One: %6.1f %6.1f %6.1f", sampleOne.getX(), sampleOne.getY(), sampleOne.getHeading()));
                // write sampleOneCorrection to telemetry
                telemetry.addLine(String.format("Sample One Correction: %6.1f %6.1f %6.1f", sampleOneCorrection.getX(), sampleOneCorrection.getY(), sampleOneCorrection.getHeading()));
                telemetry.update();


                trajSeq = drive.trajectorySequenceBuilder(sampleArea)
                        .lineToLinearHeading(sampleOneCorrection)
                        .build();


            }
            else {
                telemetry.addLine("No AprilTag Detected");
                telemetry.update();

                trajSeq = drive.trajectorySequenceBuilder(sampleArea)
                        .lineToLinearHeading(sampleOne)
                        .build();
            }

            visionPortal.close();



            // drive to sampleArea and reset pose
            drive.followTrajectorySequence(trajSeq);

            // reset pose
            drive.setPoseEstimate(sampleOne);

            // claw to neutral
            drive.clawLevelNeutral();

            // close claw around sample
            drive.closeClawAndWait();

            // raise claw level
            drive.clawLevelUp();

            // move robot to observation drop off
            trajSeq = drive.trajectorySequenceBuilder(sampleOne)
                    .lineToLinearHeading(sampleObsDrop)
                    .build();

            drive.followTrajectorySequence(trajSeq);


            // open claw to drop sample
            drive.openClaw();

            // move to observation wait
            trajSeq = drive.trajectorySequenceBuilder(sampleObsDrop)
                    .lineToLinearHeading(sampleObsWait)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            // wait for human player to swap out sample
            sleep(3000);

            // move to observation drop off
            trajSeq = drive.trajectorySequenceBuilder(sampleObsWait)
                    .lineToLinearHeading(sampleObsDrop)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            // lower claw level
            drive.clawLevelNeutral();

            // close claw to pick up sample
            drive.closeClaw();



            /*
                Wait for autonomous to finish
             */

            // stop autonomous and wait for finish
            sleep(30000);
        }
    }   // end runOpMode()
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_APRILTAG_CORRECTIONS) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()


    private void telemetryAprilTag(Telemetry telemetry) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}   // end class
