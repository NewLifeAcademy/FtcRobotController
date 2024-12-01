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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcodealpha.AlphaBot2024;
import org.firstinspires.ftc.teamcodealpha.drive.config.AlphaDriveConstants;
import org.firstinspires.ftc.teamcodealpha.trajectorysequence.TrajectorySequence;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameterConceptTensorFlowObjectDetections.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Auto - Plan Red Left Simple", preselectTeleOp = "IntoTheDeep")
@Disabled
@Config
public class AlphaPlanRedLeftSimple extends LinearOpMode {
    public static double START_POS_X = -10;
    public static double START_POS_Y = -62;
    public static double START_POS_HEADING = 90;
    public static double WAYP_ONE_X = -10;
    public static double WAYP_ONE_Y = -34;
    public static double WAYP_ONE_HEADING = 90;
    public static double WAYP_TWO_X = -10;
    public static double WAYP_TWO_Y = -48;
    public static double WAYP_TWO_HEADING = 90;
    public static double WAYP_THREE_X = -36;
    public static double WAYP_THREE_Y = -48;
    public static double WAYP_THREE_HEADING = 0;
    public static double WAYP_FOUR_X = -36;
    public static double WAYP_FOUR_Y = 6;
    public static double WAYP_FOUR_HEADING = 0;
    public static double WAYP_FIVE_X = -24;
    public static double WAYP_FIVE_Y = 6;
    public static double WAYP_FIVE_HEADING = 0;

    @Override
    public void runOpMode() {
        AlphaBot2024 drive = new AlphaBot2024(hardwareMap);

        // Create all waypoints
        Pose2d startPose = new Pose2d(START_POS_X, START_POS_Y, Math.toRadians(START_POS_HEADING));
        Pose2d waypOne = new Pose2d(WAYP_ONE_X, WAYP_ONE_Y, Math.toRadians(WAYP_ONE_HEADING));
        Pose2d waypTwo = new Pose2d(WAYP_TWO_X, WAYP_TWO_Y, Math.toRadians(WAYP_TWO_HEADING));
        Pose2d waypThree = new Pose2d(WAYP_THREE_X, WAYP_THREE_Y, Math.toRadians(WAYP_THREE_HEADING));
        Pose2d waypFour = new Pose2d(WAYP_FOUR_X, WAYP_FOUR_Y, Math.toRadians(WAYP_FOUR_HEADING));
        Pose2d waypFive = new Pose2d(WAYP_FIVE_X, WAYP_FIVE_Y, Math.toRadians(WAYP_FIVE_HEADING));

        waitForStart();

        if (opModeIsActive()) {
            drive.ClawClose.setPosition(0);
            drive.ClawExtend.setPosition(1);
            drive.ClawLevel.setPosition(0.75);

            // sleep to allow time for the claw to close to position
            sleep(2000);

            // Perform an encoder based lift
            drive.LeftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.RightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            drive.LeftLiftMotor.setTargetPosition(4000);
            drive.RightLiftMotor.setTargetPosition(4000);
            drive.LeftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.LeftLiftMotor.setPower(1);
            drive.RightLiftMotor.setPower(1);

            // Define the start pose
            drive.setPoseEstimate(startPose);
            // create a slow velocity and acceleration constraint
            TrajectoryVelocityConstraint slowVelocity = AlphaBot2024.getVelocityConstraint(36, AlphaDriveConstants.MAX_ANG_VEL, AlphaDriveConstants.TRACK_WIDTH);
            TrajectoryAccelerationConstraint slowAcceleration = AlphaBot2024.getAccelerationConstraint(36);

            //create trajectory to waypoint one and follow it
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(waypOne, slowVelocity, slowAcceleration)
                    .build();

            drive.followTrajectorySequence(trajSeq);

            while (drive.LeftLiftMotor.isBusy() && drive.RightLiftMotor.isBusy()) {
                //wait for lift to reach position
                sleep(100);
            }
            // Perform an encoder based ascent to 4600
            drive.LeftLiftMotor.setTargetPosition(4600);
            drive.RightLiftMotor.setTargetPosition(4600);
            drive.LeftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.LeftLiftMotor.setPower(.75);
            drive.RightLiftMotor.setPower(.75);

            while (drive.LeftLiftMotor.isBusy() && drive.RightLiftMotor.isBusy()) {
                //wait for lift to reach position
                sleep(100);
            }
            // create a slower velocity and acceleration constraint
            TrajectoryVelocityConstraint slowerVelocity = AlphaBot2024.getVelocityConstraint(12, AlphaDriveConstants.MAX_ANG_VEL, AlphaDriveConstants.TRACK_WIDTH);
            TrajectoryAccelerationConstraint slowerAcceleration = AlphaBot2024.getAccelerationConstraint(12);

            //create trajectory to waypoint two and follow it
            trajSeq = drive.trajectorySequenceBuilder(waypOne)
                    .lineToLinearHeading(
                            waypTwo,
                            slowerVelocity,
                            slowerAcceleration)
                    .build();


            drive.followTrajectorySequence(trajSeq);




            // open ClawClose
            drive.ClawClose.setPosition(1);



            // create trajectory to waypoint three and follow it
            trajSeq = drive.trajectorySequenceBuilder(waypTwo)
                    .lineToLinearHeading(waypThree)

                    .build();

            drive.followTrajectorySequence(trajSeq);

            // Perform an encoder based drop to 1000
            drive.LeftLiftMotor.setTargetPosition(1000);
            drive.RightLiftMotor.setTargetPosition(1000);
            drive.LeftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.RightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.LeftLiftMotor.setPower(.75);
            drive.RightLiftMotor.setPower(.75);


            // create trajectory to waypoint four and five and follow it
            trajSeq = drive.trajectorySequenceBuilder(waypThree)
                    .lineToLinearHeading(waypFour)
                    .lineToLinearHeading(waypFive, slowerVelocity, slowerAcceleration)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            // wait for lift descend to finish
            while (drive.LeftLiftMotor.isBusy() && drive.RightLiftMotor.isBusy()) {
                //wait for lift to reach position
                sleep(100);
            }

            //perform an encoder based tilt to 3000
            drive.LiftTiltMotor.setTargetPosition(3000);
            drive.LiftTiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.LiftTiltMotor.setPower(.5);
            //wait for tilt to reach position
            while (drive.LiftTiltMotor.isBusy()) {
                //wait for tilt to reach position
                sleep(100);
            }


            // stop autonomous and wait for finish
            sleep(30000);
        }
    }   // end runOpMode()
}   // end class
