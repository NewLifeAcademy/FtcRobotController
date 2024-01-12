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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcodealpha.AlphaBot2024;
import org.firstinspires.ftc.teamcodealpha.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameterConceptTensorFlowObjectDetections.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Config
public abstract class AlphaShortRedCommon extends LinearOpMode {
    public static double START_POS_X = 14.25;
    public static double START_POS_Y = -62.5;
    public static double START_POS_HEADING = 90;
    public static double WAYPOINT1_X = 30.25;
    public static double WAYPOINT1_Y = -52.5;
    public static double WAYPOINT1_HEADING = 90;
    public static double LEFT_POS_X = 20;
    public static double LEFT_POS_Y = -39;
    public static double LEFT_POS_HEADING = 0;
    public static double RIGHT_POS_X = 44;
    public static double RIGHT_POS_Y = -39;
    public static double RIGHT_POS_HEADING = 0;
    public static double CENTER_POS_X = 18.5;
    public static double CENTER_POS_Y = -44;
    public static double CENTER_POS_HEADING = 270;
    public static double WAYPOINT5_X = 36;
    public static double WAYPOINT5_Y = -39;
    public static double WAYPOINT5_HEADING = 0;

    public static double BACKBOARD_X = 55;
    public static double BACKBOARD_Y = -37;
    public static double BACKBOARD_HEADING = 0;
    public static double BACKBOARD_LEFT_OFFSET = 5;
    public static double BACKBOARD_RIGHT_OFFSET = -5;
    public static double WAYPOINT6_X = 50;
    public static double WAYPOINT6_Y = -39;
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
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    protected void opModeCode(Pose2d parkPose) {
        AlphaBot2024 drive = new AlphaBot2024(hardwareMap);

        initTfod();

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

                int pos = telemetryTfod();

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
                Pose2d dropPose, c2orLR2Pose;
                Pose2d backpose = new Pose2d(BACKBOARD_X, BACKBOARD_Y, Math.toRadians(BACKBOARD_HEADING));
                if (pos == 0) {
                    // Drive to center pose position
                    dropPose = new Pose2d(CENTER_POS_X, CENTER_POS_Y, Math.toRadians(CENTER_POS_HEADING));

                } else if (pos == 1) {
                    dropPose = new Pose2d(LEFT_POS_X, LEFT_POS_Y, Math.toRadians(LEFT_POS_HEADING));
                    backpose = new Pose2d(BACKBOARD_X , BACKBOARD_Y + BACKBOARD_LEFT_OFFSET, Math.toRadians(BACKBOARD_HEADING));
                } else {
                    dropPose = new Pose2d(RIGHT_POS_X, RIGHT_POS_Y, Math.toRadians(RIGHT_POS_HEADING));
                    backpose = new Pose2d(BACKBOARD_X , BACKBOARD_Y + BACKBOARD_RIGHT_OFFSET, Math.toRadians(BACKBOARD_HEADING));
                }
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
                TrajectorySequence seq2 = drive.trajectorySequenceBuilder(dropPose)
                        .lineToLinearHeading(new Pose2d(WAYPOINT5_X, WAYPOINT5_Y, Math.toRadians(WAYPOINT5_HEADING)))
                        .lineToLinearHeading(backpose)
                        .build();

                drive.followTrajectorySequence(seq2);

                // Raise the lift
                drive.setLiftMotorPowers(1);
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
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            // With the following lines commented out, the default TfodProcessor Builder
            // will load the default model for the season. To define a custom model to load, 
            // choose one of the following:
            //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
            //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            .setModelFileName(TFOD_MODEL_FILE)

            // The following default settings are available to un-comment and edit as needed to 
            // set parameters for custom models.
            .setModelLabels(LABELS)
            .setIsModelTensorFlow2(true)
            .setIsModelQuantized(true)
            .setModelInputSize(300)
            .setModelAspectRatio(9.0 / 5.0)

            .build();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                 .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                         .setCameraResolution(new Size(864, 480))


                .addProcessor(tfod)
                .build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private int telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

        // if number of recognitions is zero or greater than 1, display a message
        if (currentRecognitions.size() == 0 || currentRecognitions.size() > 1) {
            telemetry.addData("Image", "0 or more then 1 prop detected. Assuming center position.");
            return 0;
        }
        else {
            // get x and y of first (and only) recognition
            double y = (currentRecognitions.get(0).getTop()  + currentRecognitions.get(0).getBottom()) / 2 ;
            double x = (currentRecognitions.get(0).getLeft() + currentRecognitions.get(0).getRight()) / 2 ;
            // if y is less than 90, return 0 and display prop in center
            if (y < 250) {
                telemetry.addData("Image", "Prop detected below center line. Assuming center position.");
                return 0;
            }
            else {
                // if x is less than 400, return 1 and display prop left of center
                if (x < 400) {
                    telemetry.addData("Image", "Prop detected left of center. Assuming left position.");
                    return 1;
                }
                else {
                    // Assuming right position
                    telemetry.addData("Image", "Prop detected right of center. Assuming right position.");
                    return 2;
                }
            }
        }
    } // end method telemetryTfod()
}   // end class
