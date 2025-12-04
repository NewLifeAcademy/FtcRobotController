package org.firstinspires.ftc.teamcodeiron.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcodeiron.IronBot2025;

@Autonomous(name = "Auto - Start on Red Field - Move Only")
@Config
public class IronStartOnRedFieldMoveOnly extends LinearOpMode {

    public static double START_POSE_X = 63;
    public static double START_POSE_Y = 24;
    public static double START_POSE_HEADING = 0;
    public static double END_POSE_X = 39;
    public static double END_POSE_Y = 30;
    public static double END_POSE_HEADING = 0;

    @Override
    public void runOpMode() {
        IronBot2025 robot = new IronBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            // Starting Pose
            Pose2d startPose = new Pose2d(START_POSE_X, START_POSE_Y, Math.toRadians(START_POSE_HEADING));

            robot.localizer.setPose(startPose);

            // Move to GPP spike approach position
            Action action = robot.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d( END_POSE_X , END_POSE_Y, Math.toRadians( END_POSE_HEADING )), Math.toRadians(END_POSE_HEADING))
                    .build();
            Actions.runBlocking(new SequentialAction(action));

            // stop autonomous and wait for finish
            telemetry.addLine("Testing complete. Stopping autonomous.");
            telemetry.update();
        }
    }
}
