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

import org.firstinspires.ftc.teamcodealpha.AlphaBot2024;
import org.firstinspires.ftc.teamcodealpha.drive.config.AlphaDriveConstants;
import org.firstinspires.ftc.teamcodealpha.trajectorysequence.TrajectorySequence;

/*
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Auto - Right Side Observation", preselectTeleOp = "IntoTheDeep")

@Config
public class AlphaPlanRightObservation extends LinearOpMode {
    public static double LIFT_ASCENT_SPEED = 1;
    public static double LIFT_DESCENT_SPEED = 1;

    public static double START_POS_X = -10;
    public static double START_POS_Y = 62;
    public static double START_POS_HEADING = 270;
    public static int SUB_APPROACH_HEIGHT = 4000;
    public static int SUB_APPROACH_VELOCITY = 36;
    public static int SUB_APPROACH_ACCELERATION = 36;
    public static double SUB_APPROACH_X = -10;
    public static double SUB_APPROACH_Y = 34;
    public static double SUB_APPROACH_HEADING = 270;
    public static int SUB_FASTEN_HEIGHT = 4600;
    public static int SUB_FASTEN_VELOCITY = 12;
    public static int SUB_FASTEN_ACCELERATION = 12;
    public static double SUB_FASTEN_LIFT_SPEED = 0.75;
    public static double SUB_FASTEN_X = -10;
    public static double SUB_FASTEN_Y = 45;
    public static double SUB_FASTEN_HEADING = 270;
    public static double OBSERVE_PARK_X = -48;
    public static double OBSERVE_PARK_Y = 62;
    public static double OBSERVE_PARK_HEADING = 270;
    public static int OBSERVE_PARK_HEIGHT = 0;

    @Override
    public void runOpMode() {
        AlphaBot2024 drive = new AlphaBot2024(hardwareMap);

        // Create all waypoints
        Pose2d startPose = new Pose2d(START_POS_X, START_POS_Y, Math.toRadians(START_POS_HEADING));
        Pose2d subApproach = new Pose2d(SUB_APPROACH_X, SUB_APPROACH_Y, Math.toRadians(SUB_APPROACH_HEADING));
        Pose2d subFasten = new Pose2d(SUB_FASTEN_X, SUB_FASTEN_Y, Math.toRadians(SUB_FASTEN_HEADING));
        Pose2d observePark = new Pose2d(OBSERVE_PARK_X, OBSERVE_PARK_Y, Math.toRadians(OBSERVE_PARK_HEADING));

        // Create all velocities and accelerations
        TrajectoryVelocityConstraint subApproachVelocity = AlphaBot2024.getVelocityConstraint(SUB_APPROACH_VELOCITY, AlphaDriveConstants.MAX_ANG_VEL, AlphaDriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint subApproachAcceleration = AlphaBot2024.getAccelerationConstraint(SUB_APPROACH_ACCELERATION);
        TrajectoryVelocityConstraint subFastenVelocity = AlphaBot2024.getVelocityConstraint(SUB_FASTEN_VELOCITY, AlphaDriveConstants.MAX_ANG_VEL, AlphaDriveConstants.TRACK_WIDTH);
        TrajectoryAccelerationConstraint subFastenAcceleration = AlphaBot2024.getAccelerationConstraint(SUB_FASTEN_ACCELERATION);

        waitForStart();

        if (opModeIsActive()) {
            drive.resetLiftEncoders();
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

            // open claw
            drive.openClaw();

            /*
                Park in Observation Zone
             */

            // start lift descent
            drive.startLiftToPosition(OBSERVE_PARK_HEIGHT, LIFT_DESCENT_SPEED);

            // create trajectory to observation and follow it
            trajSeq = drive.trajectorySequenceBuilder(subFasten)
                    .lineToLinearHeading(observePark)

                    .build();

            drive.followTrajectorySequence(trajSeq);

            // wait for lift to reach position
            drive.waitForLiftToReachPosition();

            // stop autonomous and wait for finish
            sleep(30000);
        }
    }   // end runOpMode()
}   // end class
