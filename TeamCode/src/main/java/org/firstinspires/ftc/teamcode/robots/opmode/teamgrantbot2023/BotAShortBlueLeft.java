package org.firstinspires.ftc.teamcode.robots.opmode.teamgrantbot2023;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.BotA2023;
import org.firstinspires.ftc.teamcode.robots.base.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Robot A blue short left - Autonomous", preselectTeleOp = "2023-2024 IronEagle-Strafe")
public class BotAShortBlueLeft extends LinearOpMode {

    private double DISTANCE_MULTIPLIER = 1.5;

    @Override
    public void runOpMode() throws InterruptedException{
        BotA2023 drive = new BotA2023(hardwareMap);

        Pose2d startPose = new Pose2d();

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            // Close the claw
            drive.ClawServo.setPosition(-1);

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .strafeRight(DriveConstants.STRAFE_ONE_DISTANCE)
                    .strafeLeft(DriveConstants.STRAFE_TWO_DISTANCE)
                    .forward(DriveConstants.FORWARD_DISTANCE_SHORT)
                    .build();
            drive.followTrajectorySequence(trajSeq);

            // Raise the lift
            drive.LeftLiftMotor.setPower(-1);
            drive.RightLiftMotor.setPower(-1);

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
                    .strafeLeft(DriveConstants.STRAFE_THREE_DISTANCE)
                    .build();
            drive.followTrajectorySequence(trajSeq);
            sleep(30000);

        }
    }
}
