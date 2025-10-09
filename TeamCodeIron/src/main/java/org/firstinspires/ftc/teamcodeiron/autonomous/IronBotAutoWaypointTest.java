package org.firstinspires.ftc.teamcodeiron.autonomous;

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
import org.firstinspires.ftc.teamcodeiron.IronBot2025;

@Autonomous(name = "Auto - Waypoint Test")
@Config
public class IronBotAutoWaypointTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        IronBot2025 robot = new IronBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            // Waypoint test
            Action action = robot.actionBuilder(new Pose2d(0, 0, 0))
                    .waitSeconds(2)
                    .setTangent(0)
                    .splineTo(new Vector2d(-24,24), Math.toRadians(315))
                    .waitSeconds(1)

                    .build();
            Actions.runBlocking(new SequentialAction(action));

            // AprilTag detection test
            robot.initAprilTagDetection();
            robot.telemetryAprilTag(telemetry);

            // stop autonomous and wait for finish
            telemetry.addLine("Testing complete. Stopping autonomous.");
            telemetry.update();
        }
    }
}
