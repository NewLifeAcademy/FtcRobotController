package org.firstinspires.ftc.teamcodeiron.autonomous.common;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodeiron.IronBot2025;

public class IronBotCommonSequences {
    public static void RunStartOnGoalSequence(
            IronBot2025 robot,
            Telemetry telemetry,
            double start_pose_x,
            double start_pose_y,
            double start_heading,
            double waypoint_fire_01_x,
            double waypoint_fire_01_y,
            double waypoint_fire_01_heading,
            double spike_approach_x,
            double spike_approach_y,
            double spike_approach_heading,
            double waypoint_fire_02_x,
            double waypoint_fire_02_y,
            double waypoint_fire_02_heading,
            double end_pose_x,
            double end_pose_y,
            double end_heading
    ) {
        // Starting Pose
        Pose2d startPose = new Pose2d(start_pose_x, start_pose_y, Math.toRadians(start_heading));

        robot.localizer.setPose(startPose);

        // Move to near launch position
        Action action = robot.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d( waypoint_fire_01_x , waypoint_fire_01_y, Math.toRadians( waypoint_fire_01_heading) ), Math.toRadians( waypoint_fire_01_heading ))
                .build();
        Actions.runBlocking(new SequentialAction(action));

        // Fire three preloaded artifacts
        robot.firePreloadedArtifacts();

        // TODO: Use camera telemetry to determine target spike mark:
        // - Far (Goal Side) = PPG
        // - Middle (Center) = PGP
        // - Near (Audience Side) = GPP

        // Move to PPG spike approach position
        action = robot.actionBuilder(robot.localizer.getPose())
                .splineToLinearHeading(new Pose2d( spike_approach_x , spike_approach_y, Math.toRadians(spike_approach_heading) ), Math.toRadians( spike_approach_heading ))
                .build();
        Actions.runBlocking(new SequentialAction(action));

        // TODO: cooridnate intake and spinarizer to load three artifacts from spike mark

        // Move to near launch position
        action = robot.actionBuilder(robot.localizer.getPose())
                .splineToLinearHeading(new Pose2d( waypoint_fire_02_x , waypoint_fire_02_y, Math.toRadians(waypoint_fire_02_heading) ), Math.toRadians( waypoint_fire_02_heading ))
                .build();

        Actions.runBlocking(new SequentialAction(action));


        // Move to End pose position
        action = robot.actionBuilder(robot.localizer.getPose())
                .splineToLinearHeading(new Pose2d( end_pose_x , end_pose_y, Math.toRadians(end_heading)), Math.toRadians( end_heading ))
                .build();
        Actions.runBlocking(new SequentialAction(action));

        // stop autonomous and wait for finish
        telemetry.addLine("Testing complete. Stopping autonomous.");
        telemetry.update();

    }

    public static void RunStartOnFieldFireSequence(
            IronBot2025 robot,
            Telemetry telemetry,
            double start_pose_x,
            double start_pose_y,
            double start_heading,
            double waypoint_fire_x,
            double waypoint_fire_y,
            double waypoint_fire_heading,
            double end_pose_x,
            double end_pose_y,
            double end_heading
    ) {
        // Starting Pose
        Pose2d startPose = new Pose2d(start_pose_x, start_pose_y, Math.toRadians(start_heading));

        robot.localizer.setPose(startPose);

        // Move to far launch position
        Action action = robot.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(waypoint_fire_x, waypoint_fire_y, Math.toRadians(waypoint_fire_heading)), Math.toRadians(waypoint_fire_heading))
                .build();
        Actions.runBlocking(new SequentialAction(action));

        // Fire preloaded
        robot.firePreloadedArtifacts();

        // Move to GPP spike approach position
        action = robot.actionBuilder(robot.localizer.getPose())
                .splineToLinearHeading(new Pose2d(end_pose_x, end_pose_y, Math.toRadians(end_heading)), Math.toRadians(end_heading))
                .build();
        Actions.runBlocking(new SequentialAction(action));

        // stop autonomous and wait for finish
        telemetry.addLine("Testing complete. Stopping autonomous.");

    }

    public static void RunStartOnFieldMoveOnlySequence(
            IronBot2025 robot,
            Telemetry telemetry,
            double start_pose_x,
            double start_pose_y,
            double start_heading,
            double end_pose_x,
            double end_pose_y,
            double end_heading
    ) {
        // Starting Pose
        Pose2d startPose = new Pose2d(start_pose_x, start_pose_y, Math.toRadians(start_heading));

        robot.localizer.setPose(startPose);

        // Move to GPP spike approach position
        Action action = robot.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(end_pose_x, end_pose_y, Math.toRadians(end_heading)), Math.toRadians(end_heading))
                .build();
        Actions.runBlocking(new SequentialAction(action));

        // stop autonomous and wait for finish
        telemetry.addLine("Testing complete. Stopping autonomous.");
        telemetry.update();
    }
}
