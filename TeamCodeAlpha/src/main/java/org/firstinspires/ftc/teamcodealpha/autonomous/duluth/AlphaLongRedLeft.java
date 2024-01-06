package org.firstinspires.ftc.teamcodealpha.autonomous.duluth;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcodealpha.AlphaBot2024;
import org.firstinspires.ftc.teamcodealpha.drive.config.AlphaDriveConstants;
import org.firstinspires.ftc.teamcodealpha.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Old robot A red long left - Autonomous", preselectTeleOp = "2023-2024 IronEagle-Strafe")
@Disabled
public class AlphaLongRedLeft extends LinearOpMode {

    private double DISTANCE_MULTIPLIER = 1.5;

    @Override
    public void runOpMode() throws InterruptedException{
        AlphaBot2024 drive = new AlphaBot2024(hardwareMap);

        Pose2d startPose = new Pose2d();

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            // 5 sec delay to allow for alliance partner to move
            sleep(5000);

            // Close the claw
            drive.ClawServo.setPosition(-1);

            // TODO: Add vision support to recognize placement of the team prop, and then adjust
            //  values to push purple pixel to the correct spot in the first drive sequence

            // TODO: (Optional) After adding vision support, add AprilTag detection to create a
            //  'closing distance trajectory' to move the robot closer to the correct spot on the board.

            // Drive sequence to push pixel and move to board
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    // TODO: Tune and fix the negative distance values
                    .strafeLeft(AlphaDriveConstants.STRAFE_ONE_RED_DISTANCE)
                    .strafeRight(AlphaDriveConstants.STRAFE_TWO_RED_DISTANCE)
                    .forward(AlphaDriveConstants.FORWARD_DISTANCE_RED_LONG)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            // Raise the lift
            drive.LeftLiftMotor.setPower(-1);
            drive.RightLiftMotor.setPower(-1);

            // TODO: Add motor encoders to lift motors to use number of motor rotations instead of time
            sleep(1000);

            // Stop the lift
            drive.LeftLiftMotor.setPower(0);
            drive.RightLiftMotor.setPower(0);

            // Lower the claw
            drive.ClawLiftServo.setPosition(-1);
            sleep(1000);

            // Open the claw
            drive.ClawServo.setPosition(0.8);
            sleep(1000);

            // Raise the claw
            drive.ClawLiftServo.setPosition(1);
            sleep(1000);

            trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .back(AlphaDriveConstants.REVERSE_DISTANCE)
                    .strafeLeft(AlphaDriveConstants.STRAFE_THREE_DISTANCE)
                    .build();
            drive.followTrajectorySequence(trajSeq);
            sleep(30000);

        }
    }
}
