package org.firstinspires.ftc.teamcodebeta.robots.opmode.robotbeta;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcodebeta.robots.BetaBot2024;
import org.firstinspires.ftc.teamcodebeta.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Robot B - Autonomous")
@Disabled
public class BetaAutonomous extends LinearOpMode {

    public static double DISTANCE = 10; // in
    @Override
    public void runOpMode() throws InterruptedException{
        BetaBot2024 drive = new BetaBot2024(hardwareMap);


        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .build();
            drive.followTrajectorySequence(trajSeq);


        }
    }
}
