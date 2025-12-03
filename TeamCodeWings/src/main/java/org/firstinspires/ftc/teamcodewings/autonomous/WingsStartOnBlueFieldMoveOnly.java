package org.firstinspires.ftc.teamcodewings.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodewings.WingsBot2025;

@Autonomous(name = "Auto - Start on Blue Field - Move Only")
@Config
public class WingsStartOnBlueFieldMoveOnly extends LinearOpMode {

    public static double START_POSE_X= 63;
    public static double START_POSE_Y=-24;
    public static double END_POSE_X=39;
    public static double END_POSE_Y=-30;

    @Override
    public void runOpMode() {
        WingsBot2025 robot = new WingsBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            // Starting Pose
            Pose2d startPose = new Pose2d(START_POSE_X, START_POSE_Y, Math.toRadians(180));

            robot.localizer.setPose(startPose);

            // Build action sequence
            Action action = robot.actionBuilder(startPose)
                    .splineTo(new Vector2d( END_POSE_X , END_POSE_Y), Math.toRadians( 180 ))
                    .build();
            Actions.runBlocking(new SequentialAction(action));

            // stop autonomous and wait for finish
            telemetry.addLine("Testing complete. Stopping autonomous.");
            telemetry.update();
        }
    }
}
