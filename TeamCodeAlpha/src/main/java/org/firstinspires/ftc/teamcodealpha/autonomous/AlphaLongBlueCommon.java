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

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcodealpha.AlphaBot2024;
import org.firstinspires.ftc.teamcodealpha.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameterConceptTensorFlowObjectDetections.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Config
public abstract class AlphaLongBlueCommon extends LinearOpMode {
    public static double START_POS_X = -36.25;
    public static double START_POS_Y = 62.5;
    public static double START_POS_HEADING = 270;
    public static double WAYPOINT1_X = -42.25;
    public static double WAYPOINT1_Y = 52.5;
    public static double WAYPOINT1_HEADING = 270;
    public static double LEFT_POS_X = -43.25;
    public static double LEFT_POS_Y = 42.25;
    public static double LEFT_POS_HEADING = 180;
    public static double RIGHT_POS_X = -28.75;
    public static double RIGHT_POS_Y = 36.75;
    public static double RIGHT_POS_HEADING = 0;
    public static double CENTER_POS_X = -41;
    public static double CENTER_POS_Y = 44.5;
    public static double CENTER_POS_HEADING = 90;
    public static double CENTER2_X = -55;
    public static double CENTER2_Y = 43.5;
    public static double CENTER2_HEADING = 0;
    public static double LR2_X = -40.25;
    public static double LR2_Y = 11;
    public static double LR2_HEADING = 0;
    public static double WAYPOINT3_X = -55;
    public static double WAYPOINT3_Y = 11;
    public static double WAYPOINT3_HEADING = 0;
    public static boolean USE_SPLINE_FOR_W4_TO_BACKBOARD = false;
    public static double USE_SPLINE_TANGENT = 270;

    public static double WAYPOINT4_X = 36;
    public static double WAYPOINT4_Y = 11;
    public static double WAYPOINT4_HEADING = 0;
    public static double WAYPOINT5_X = 36;
    public static double WAYPOINT5_Y = 36;
    public static double WAYPOINT5_HEADING = 0;

    public static double BACKBOARD_X = 54;
    public static double BACKBOARD_Y = 36;
    public static double BACKBOARD_HEADING = 0;
    public static double BACKBOARD_LEFT_OFFSET = 6;
    public static double BACKBOARD_RIGHT_OFFSET = -6;
    public static double WAYPOINT6_X = 50;
    public static double WAYPOINT6_Y = 36;
    public static double WAYPOINT6_HEADING = 0;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_centerstage1.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
       "prop",
    };

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    protected void opModeCode(Pose2d parkPose) {
        AlphaBot2024 drive = new AlphaBot2024(hardwareMap);

        initVisionPortal();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            drive.claw2close.setPosition(-1);
            drive.ClawServo.setPosition(-1);
            while (opModeIsActive()) {
                // 4 second sleep to allow TFOD to detect prop
                sleep(4000);

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources
                visionPortal.close();

                // set starting position of robot
                Pose2d startPose = new Pose2d(START_POS_X, START_POS_Y, Math.toRadians(START_POS_HEADING));
                drive.setPoseEstimate(startPose);

                // Setup waypoint1
                Pose2d waypoint1 = new Pose2d(WAYPOINT1_X, WAYPOINT1_Y, Math.toRadians(WAYPOINT1_HEADING));

                // if pos is 0 then move to center position, rotate 180 degrees, lower rear claw, open rear claw, raise rear claw
                Pose2d dropPose, c2orLR2Pose, backpose;
                dropPose = new Pose2d(RIGHT_POS_X, RIGHT_POS_Y, Math.toRadians(RIGHT_POS_HEADING));
                c2orLR2Pose = new Pose2d(LR2_X, LR2_Y, Math.toRadians(LR2_HEADING));
                backpose = new Pose2d(BACKBOARD_X , BACKBOARD_Y + BACKBOARD_RIGHT_OFFSET, Math.toRadians(BACKBOARD_HEADING));

                // Move to dropPose
                TrajectorySequence seq1 = drive.trajectorySequenceBuilder(startPose)
                        .lineToLinearHeading(waypoint1)
                        .lineToLinearHeading(dropPose)
                        .build();

                drive.followTrajectorySequence(seq1);

                // Lower claw2
                drive.claw2flip.setPosition(0.8);
                sleep(500);

                // Drop pixel
                drive.claw2close.setPosition(0.8);
                sleep(500);

                // Raise claw2
                drive.claw2flip.setPosition(-1);
                sleep(500);

                // Close claw2
                drive.claw2close.setPosition(-1);
                sleep(500);

                // Move to backfield
                TrajectorySequence seq2;
                if (USE_SPLINE_FOR_W4_TO_BACKBOARD) {
                    // Spline option
                    seq2 = drive.trajectorySequenceBuilder(dropPose)
                            .lineToLinearHeading(c2orLR2Pose)
                            .lineToLinearHeading(new Pose2d(WAYPOINT3_X, WAYPOINT3_Y, Math.toRadians(WAYPOINT3_HEADING)))
                            .lineToLinearHeading(new Pose2d(WAYPOINT4_X, WAYPOINT4_Y, Math.toRadians(WAYPOINT4_HEADING)))
                            .splineTo(new Vector2d(backpose.getX(), backpose.getY()), Math.toRadians(backpose.getHeading()))
                            .build();
                } else {
                    // Linear option
                    seq2 = drive.trajectorySequenceBuilder(dropPose)
                            .lineToLinearHeading(c2orLR2Pose)
                            .lineToLinearHeading(new Pose2d(WAYPOINT3_X, WAYPOINT3_Y, Math.toRadians(WAYPOINT3_HEADING)))
                            .lineToLinearHeading(new Pose2d(WAYPOINT4_X, WAYPOINT4_Y, Math.toRadians(WAYPOINT4_HEADING)))
                            .lineToLinearHeading(new Pose2d(WAYPOINT5_X, WAYPOINT5_Y, Math.toRadians(WAYPOINT5_HEADING)))
                            .lineToLinearHeading(backpose)
                            .build();
                }

                drive.followTrajectorySequence(seq2);

                // Raise the lift
                drive.setLiftMotorPowers(0.6);
                sleep(1000);

                // Stop the lift
                drive.setLiftMotorPowers(0);
                sleep(500);

                // Lower the claw
                drive.ClawLiftServo.setPosition(-1);
                sleep(500);

                // Open the claw
                drive.ClawServo.setPosition(0.8);
                sleep(500);

                // Raise the claw
                drive.ClawLiftServo.setPosition(1);
                sleep(500);

                // Move to park
                // TODO: Change heading to avoid any 'strafing' (seems to consume too much battery)
                TrajectorySequence seq3 = drive.trajectorySequenceBuilder(backpose)
                        .lineToLinearHeading(new Pose2d(WAYPOINT6_X, WAYPOINT6_Y, Math.toRadians(WAYPOINT6_HEADING)))
                        .lineToLinearHeading(parkPose)
                        .build();

                drive.followTrajectorySequence(seq3);

                // stop autonomous and wait for finish
                sleep(30000);
            }
        }
    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initVisionPortal() {

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                 .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                         .setCameraResolution(new Size(864, 480))
                .build();
    }   // end method initVisionPortal()
}   // end class
