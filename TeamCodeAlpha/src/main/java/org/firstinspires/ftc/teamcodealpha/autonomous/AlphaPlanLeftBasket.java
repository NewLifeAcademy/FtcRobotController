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
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcodealpha.AlphaBot2024;
import org.firstinspires.ftc.teamcodealpha.drive.config.AlphaDriveConstants;
import org.firstinspires.ftc.teamcodealpha.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Auto - Left Side Basket", preselectTeleOp = "IntoTheDeep")

@Config
public class AlphaPlanLeftBasket extends LinearOpMode {
    public static double LIFT_ASCENT_SPEED = 1;
    public static double LIFT_DESCENT_SPEED = 1;
    public static double TILT_SPEED = 1;

    public static double START_POS_X = 10;
    public static double START_POS_Y = 62;
    public static double START_POS_HEADING = 270;
    public static double SUB_APPROACH_X = 10;
    public static double SUB_APPROACH_Y = 32;
    public static double SUB_APPROACH_HEADING = 270;
    public static int SUB_APPROACH_HEIGHT = 4000;
    public static int SUB_APPROACH_VELOCITY = 36;
    public static int SUB_APPROACH_ACCELERATION = 36;
    public static double SUB_FASTEN_X = 10;
    public static double SUB_FASTEN_Y = 45;
    public static double SUB_FASTEN_HEADING = 270;
    public static int SUB_FASTEN_HEIGHT = 4600;
    public static int SUB_FASTEN_VELOCITY = 12;
    public static int SUB_FASTEN_ACCELERATION = 12;
    public static double SUB_FASTEN_LIFT_SPEED = 0.75;
    public static int SUB_FASTEN_REVERSE_HEIGHT = 4900;
    public static double SAMPLE_AREA_ONE_X = 48;
    public static double SAMPLE_AREA_ONE_Y = 48;
    public static double SAMPLE_AREA_ONE_HEADING = 270;
    public static int SAMPLE_LIFT_HEIGHT = 125;
    public static double SAMPLE_ONE_X = 48;
    public static double SAMPLE_ONE_Y = 36;
    public static double SAMPLE_ONE_HEADING = 270;
    public static double UPPER_BASKET_DROP_X = 56;
    public static double UPPER_BASKET_DROP_Y = 53;
    public static double UPPER_BASKET_DROP_HEADING = 47;
    public static int UPPER_BASKET_DROP_HEIGHT = 6300;
    public static double LOWER_BASKET_DROP_X = 56;
    public static double LOWER_BASKET_DROP_Y = 54;
    public static double LOWER_BASKET_DROP_HEADING = 47;
    public static int LOWER_BASKET_DROP_HEIGHT = 2800;
    public static double BASKET_APPROACH_X = 47;
    public static double BASKET_APPROACH_Y = 47;
    public static double BASKET_APPROACH_HEADING = 37;
    public static double SAMPLE_AREA_TWO_X = 58;
    public static double SAMPLE_AREA_TWO_Y = 48;
    public static double SAMPLE_AREA_TWO_HEADING = 270;
    public static double SAMPLE_TWO_X = 58;
    public static double SAMPLE_TWO_Y = 36;
    public static double SAMPLE_TWO_HEADING = 270;
    private static final boolean USE_APRILTAG_CORRECTIONS = true;  // true for to use webcam and april tag detection for micro adjustments

    private static final double APRILTAG_TARGET_Y = 16.35;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        AlphaBot2024 drive = new AlphaBot2024(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        initAprilTag();
        // Create all waypoints
        Pose2d startPose = new Pose2d(START_POS_X, START_POS_Y, Math.toRadians(START_POS_HEADING));
        Pose2d subApproach = new Pose2d(SUB_APPROACH_X, SUB_APPROACH_Y, Math.toRadians(SUB_APPROACH_HEADING));
        Pose2d subFasten = new Pose2d(SUB_FASTEN_X, SUB_FASTEN_Y, Math.toRadians(SUB_FASTEN_HEADING));
        Pose2d sampleAreaOne = new Pose2d(SAMPLE_AREA_ONE_X, SAMPLE_AREA_ONE_Y, Math.toRadians(SAMPLE_AREA_ONE_HEADING));
        Pose2d sampleOne = new Pose2d(SAMPLE_ONE_X, SAMPLE_ONE_Y, Math.toRadians(SAMPLE_ONE_HEADING));
        Pose2d upperBasketDrop = new Pose2d(UPPER_BASKET_DROP_X, UPPER_BASKET_DROP_Y, Math.toRadians(UPPER_BASKET_DROP_HEADING));
        Pose2d lowerBasketDrop = new Pose2d(LOWER_BASKET_DROP_X, LOWER_BASKET_DROP_Y, Math.toRadians(LOWER_BASKET_DROP_HEADING));
        Pose2d basketApproach = new Pose2d(BASKET_APPROACH_X, BASKET_APPROACH_Y, Math.toRadians(BASKET_APPROACH_HEADING));
        Pose2d sampleAreaTwo = new Pose2d(SAMPLE_AREA_TWO_X, SAMPLE_AREA_TWO_Y, Math.toRadians(SAMPLE_AREA_TWO_HEADING));
        Pose2d sampleTwo = new Pose2d(SAMPLE_TWO_X, SAMPLE_TWO_Y, Math.toRadians(SAMPLE_TWO_HEADING));

        // Create all velocities and accelerations
        TrajectoryVelocityConstraint subApproachVelocity = AlphaBot2024.getVelocityConstraint(SUB_APPROACH_VELOCITY, AlphaDriveConstants.MAX_ANG_VEL, AlphaDriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint subApproachAcceleration = AlphaBot2024.getAccelerationConstraint(SUB_APPROACH_ACCELERATION);
        TrajectoryVelocityConstraint subFastenVelocity = AlphaBot2024.getVelocityConstraint(SUB_FASTEN_VELOCITY, AlphaDriveConstants.MAX_ANG_VEL, AlphaDriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint subFastenAcceleration = AlphaBot2024.getAccelerationConstraint(SUB_FASTEN_ACCELERATION);

        waitForStart();

        if (opModeIsActive()) {
            drive.resetLiftEncoders();
            drive.resetTiltEncoder();
            drive.closeClawAndWait();
            drive.retractClawArm();

            // Define the starting point
            drive.setPoseEstimate(startPose);

            /*
                Move into submersible
            */

            // Raise list to approach height
            drive.startLiftToPosition(SUB_APPROACH_HEIGHT, LIFT_ASCENT_SPEED);

            // create trajectory to submersible approach with custom velocity
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(
                            subApproach,
                            subApproachVelocity,
                            subApproachAcceleration)
                    .build();

            drive.followTrajectorySequence(trajSeq);

            // wait for lift to reach position
            drive.waitForLiftToReachPosition();

            /*
                Raise specimen in submersible
            */

            // Raise specimen to fasten to submersible
            drive.startLiftToPosition(SUB_FASTEN_HEIGHT, SUB_FASTEN_LIFT_SPEED);

            // wait for lift to reach position
            drive.waitForLiftToReachPosition();

            /*
                Reverse out of submersible
            */

            // create trajectory to subFasten with custom velocity and follow it
            trajSeq = drive.trajectorySequenceBuilder(subApproach)
                    .lineToLinearHeading(
                            subFasten,
                            subFastenVelocity,
                            subFastenAcceleration)
                    .build();

            drive.followTrajectorySequence(trajSeq);

            drive.startLiftToPosition(SUB_FASTEN_REVERSE_HEIGHT, SUB_FASTEN_LIFT_SPEED);

            // open claw
            drive.openClaw();

            /*
                Move to Sample One
            */

            // lower lift to sample height
            drive.startLiftToPosition(SAMPLE_LIFT_HEIGHT, LIFT_DESCENT_SPEED);

            // create trajectory to sampleAreaOne
            trajSeq = drive.trajectorySequenceBuilder(subFasten)
                    .lineToLinearHeading(sampleAreaOne)
                    .build();

            drive.followTrajectorySequence(trajSeq);

            // Use AprilTag for heading correction
            telemetryAprilTag(telemetry);

            // Push telemetry to the Driver Station.
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

                //set values of sampleAreaOne based on detection
                Pose2d sampleOneCorrection = new Pose2d(sampleOne.getX()-(APRILTAG_TARGET_Y-y), sampleOne.getY(), sampleOne.getHeading() - Math.toRadians(yaw));


                // write detection to telemetry
                telemetry.addLine(String.format("April Tag Detection: %6.1f %6.1f %6.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw));
                // write sampleOneCorrection to telemetry
                telemetry.addLine(String.format("Sample One: %6.1f %6.1f %6.1f", sampleOne.getX(), sampleOne.getY(), sampleOne.getHeading()));
                // write sampleOneCorrection to telemetry
                telemetry.addLine(String.format("Sample One Correction: %6.1f %6.1f %6.1f", sampleOneCorrection.getX(), sampleOneCorrection.getY(), sampleOneCorrection.getHeading()));
                telemetry.update();


                trajSeq = drive.trajectorySequenceBuilder(sampleAreaOne)
                        .lineToLinearHeading(sampleOneCorrection)
                        .build();

            }
            else {
                telemetry.addLine("No AprilTag Detected");
                telemetry.update();

                trajSeq = drive.trajectorySequenceBuilder(sampleAreaOne)
                        .lineToLinearHeading(sampleOne)
                        .build();
            }


            drive.followTrajectorySequence(trajSeq);
            drive.setPoseEstimate(sampleOne);

            // wait for lift descent to finish
            drive.waitForLiftToReachPosition();


            // close claw around sampleOne
            drive.closeClawAndWait();
            sleep(200);

            /*
                Lift and Move to Basket Approach
            */
            // lift to basket drop height
            drive.startLiftToPosition(UPPER_BASKET_DROP_HEIGHT, LIFT_ASCENT_SPEED);

            // extend the claw arm
            drive.extendClawArm();

            // create trajectory to basket drop and follow it
            trajSeq = drive.trajectorySequenceBuilder(sampleOne)
                    .lineToLinearHeading(basketApproach)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            // wait for lift to reach position
            drive.waitForLiftToReachPosition();

            /*
                Drop Sample One in High Basket
            */

            // move to basket drop position
            trajSeq = drive.trajectorySequenceBuilder(basketApproach)
                    .lineToLinearHeading(upperBasketDrop)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            //open claw (release sample)
            drive.openClaw();


            // wait (allow sample to drop in basket)
            sleep(200);

            /*
                Move to Sample Area for Sample Two
            */

            // Move to basket approach
            trajSeq = drive.trajectorySequenceBuilder(upperBasketDrop)
                    .lineToLinearHeading(sampleAreaTwo)
                    .build();

            drive.followTrajectorySequence(trajSeq);

            // retract the claw arm
            drive.retractClawArm();

            // Lower lift to sample height
            drive.startLiftToPosition(SAMPLE_LIFT_HEIGHT, LIFT_DESCENT_SPEED);

            drive.waitForLiftToReachPosition();

            /*
                Move to Sample Two
            */

            // Move to sample two
            trajSeq = drive.trajectorySequenceBuilder(sampleAreaTwo)
                    .lineToLinearHeading(sampleTwo)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            //close claw around sampleTwo
            drive.closeClawAndWait();
            sleep(200);

            /*
                Lift and Move to Basket Approach
            */
            // lift to basket drop height
            drive.startLiftToPosition(LOWER_BASKET_DROP_HEIGHT, LIFT_ASCENT_SPEED);

            // extend the claw arm
            drive.extendClawArm();

            // create trajectory to basket drop and follow it
            trajSeq = drive.trajectorySequenceBuilder(sampleTwo)
                    .lineToLinearHeading(basketApproach)
                    .build();
            drive.followTrajectorySequence(trajSeq);


            // wait for lift to reach position
            drive.waitForLiftToReachPosition();

            /*
                Drop Sample Two in High Basket
            */

            // move to basket drop position
            trajSeq = drive.trajectorySequenceBuilder(basketApproach)
                    .lineToLinearHeading(lowerBasketDrop)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            //open claw (release sample)
            drive.openClaw();


            /*
                Move to Basket approach, and setup for teleop (lower lift and retract claw)
             */

            // move to basket approach
            trajSeq = drive.trajectorySequenceBuilder(lowerBasketDrop)
                    .lineToLinearHeading(basketApproach)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            // retract the claw arm
            drive.retractClawArm();

            // lower lift to starting height
            drive.startLiftToPosition(0, LIFT_DESCENT_SPEED);

            // wait for lift to reach position
            drive.waitForLiftToReachPosition();

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
