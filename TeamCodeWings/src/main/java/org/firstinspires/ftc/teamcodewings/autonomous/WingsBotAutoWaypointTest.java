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
import org.firstinspires.ftc.teamcode.Position;
import org.firstinspires.ftc.teamcodewings.WingsBot2025;

@Autonomous(name = "Auto - Waypoint Test")
@Config
public class WingsBotAutoWaypointTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        WingsBot2025 robot = new WingsBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            // Starting Pose
            Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

            robot.localizer.setPose(startPose);

            // Build action sequence
            Action action = robot.actionBuilder(startPose)
                    .splineTo(new Vector2d(24, 0), Math.toRadians(0))
                    .build();
            // End action sequence

            // Run the action sequence
            Actions.runBlocking(new SequentialAction(action));

            // stop autonomous and wait for finish
            telemetry.addLine("Testing complete. Stopping autonomous.");
            telemetry.update();
        }
    }
}
