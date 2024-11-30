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
@Autonomous(name = "Auto - Plan Red Left 2", preselectTeleOp = "IntoTheDeep")

@Config
public class AlphaPlanRedLeft2 extends LinearOpMode {
    public static int SUMBERSIBLE_APPROACH_HEIGHT = 4000;
    public static int SUBMERSIBLE_REVERSE_HEIGHT = 4600;
    public static int BASKET_DROP_HEIGHT = 6300;
    public static int FLOOR_LIFT_HEIGHT = 0;
    public static double LIFT_ASCENT_SPEED = 1;
    public static double LIFT_DESCENT_SPEED = 1;

    public static double START_POS_X = -10;
    public static double START_POS_Y = -62;
    public static double START_POS_HEADING = 90;
    public static double WAYP_ONE_X = -10;
    public static double WAYP_ONE_Y = -34;
    public static double WAYP_ONE_HEADING = 90;
    public static double WAYP_TWO_X = -10;
    public static double WAYP_TWO_Y = -45;
    public static double WAYP_TWO_HEADING = 90;
    public static double WAYP_THREE_X = -48;
    public static double WAYP_THREE_Y = -48;
    public static double WAYP_THREE_HEADING = 90;
    public static double WAYP_FOUR_X = -48;
    public static double WAYP_FOUR_Y = -35;
    public static double WAYP_FOUR_HEADING = 90;
    public static double WAYP_FIVE_X = -55;
    public static double WAYP_FIVE_Y = -55;
    public static double WAYP_FIVE_HEADING = 230;
    public static double WAYP_SIX_X = -51;
    public static double WAYP_SIX_Y = -51;
    public static double WAYP_SIX_HEADING = 230;
    public static double WAYP_SEVEN_X = -59;
    public static double WAYP_SEVEN_Y = -37;
    public static double WAYP_SEVEN_HEADING = 90;
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
        Pose2d waypSix = new Pose2d(WAYP_SIX_X, WAYP_SIX_Y, Math.toRadians(WAYP_SIX_HEADING));
        Pose2d waypSeven = new Pose2d(WAYP_SEVEN_X, WAYP_SEVEN_Y, Math.toRadians(WAYP_SEVEN_HEADING));

        waitForStart();

        if (opModeIsActive()) {
            drive.resetLiftEncoders();
            drive.closeClawAndWait();
            drive.retractClawArm();

        /*  Move into submersible */
            // Lift to 4000 with power 1
            drive.startLiftToPosition(SUMBERSIBLE_APPROACH_HEIGHT, LIFT_ASCENT_SPEED);

            // Define the starting point
            drive.setPoseEstimate(startPose);

            // create a slow velocity and acceleration constraint
            TrajectoryVelocityConstraint slowVelocity = AlphaBot2024.getVelocityConstraint(36, AlphaDriveConstants.MAX_ANG_VEL, AlphaDriveConstants.TRACK_WIDTH);
            TrajectoryAccelerationConstraint slowAcceleration = AlphaBot2024.getAccelerationConstraint(36);

            //create trajectory to waypoint one and follow it
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(waypOne, slowVelocity, slowAcceleration)
                    .build();

            drive.followTrajectorySequence(trajSeq);

            // wait for lift to reach position
            drive.waitForLiftToReachPosition();

        /* Raise specimen in submersible */
            // Lift to 4600 with power .75
            drive.startLiftToPosition(SUBMERSIBLE_REVERSE_HEIGHT, .75);

            // wait for lift to reach position
            drive.waitForLiftToReachPosition();

        /* Reverse out of submersible */
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

            // open claw
            drive.openClaw();

            // lift to position 0 with power .75
            drive.startLiftToPosition(FLOOR_LIFT_HEIGHT, LIFT_DESCENT_SPEED);

            // create trajectory to waypoint three and follow it
            trajSeq = drive.trajectorySequenceBuilder(waypTwo)
                    .lineToLinearHeading(waypThree)
                    .lineToLinearHeading(waypFour)
                    .build();

            drive.followTrajectorySequence(trajSeq);

            // wait for lift descend to finish
            drive.waitForLiftToReachPosition();

            // close claw
            drive.closeClawAndWait();

            // extend the claw arm
            drive.extendClawArm();

            // lift to basket drop height
            drive.startLiftToPosition(BASKET_DROP_HEIGHT, LIFT_ASCENT_SPEED);

            // wait for lift to reach position
            drive.waitForLiftToReachPosition();

            // create trajectory to waypoint six and follow it
            trajSeq = drive.trajectorySequenceBuilder(waypFour)
                    .lineToLinearHeading(waypFive)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            //open claw
            drive.openClaw();

            // wait (allow sample to drop in basket)
            sleep(200);

            // move from waypoint five to waypoint six
            trajSeq = drive.trajectorySequenceBuilder(waypFive)
                    .lineToLinearHeading(waypSix)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            // lift to position 0 with power .75
            drive.startLiftToPosition(FLOOR_LIFT_HEIGHT, LIFT_DESCENT_SPEED);

            // retract the claw arm
            drive.retractClawArm();

            // create trajectory to waypoint six and follow it
            trajSeq = drive.trajectorySequenceBuilder(waypSix)
                    .lineToLinearHeading(waypSeven)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            // wait for lift descend to finish
            drive.waitForLiftToReachPosition();

            // close claw
            drive.closeClawAndWait();

            // extend the claw arm
            drive.extendClawArm();

            // lift to basket drop position
            drive.startLiftToPosition(BASKET_DROP_HEIGHT, LIFT_ASCENT_SPEED);

            // wait for lift to reach position
            drive.waitForLiftToReachPosition();

            // create trajectory to waypoint six and follow it
            trajSeq = drive.trajectorySequenceBuilder(waypSeven)
                    .lineToLinearHeading(waypFive)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            //open claw
            drive.openClaw();

            // stop autonomous and wait for finish
            sleep(30000);
        }
    }   // end runOpMode()
}   // end class
