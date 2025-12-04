package org.firstinspires.ftc.teamcodewings.autonomous;

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
import org.firstinspires.ftc.teamcodewings.WingsBot2025;

@Autonomous(name = "Auto - Start on Red Field - Fire")
@Config
public class WingsStartOnRedFieldFire extends LinearOpMode {

    public static int FIRE_TIME = 5;
    public static int FLYWHEEL_SPINUP_TIME = 2;
    public static double FLYWHEEL_POWER = 1;
    public static double START_POSE_X = 63;
    public static double START_POSE_Y = 24;
    public static double START_HEADING = 0;
    public static double WAYPOINT_FIRE_X = 40;
    public static double WAYPOINT_FIRE_Y = -1;
    public static double WAYPOINT_FIRE_HEADING = 330;
    public static double END_POSE_X = 39;
    public static double END_POSE_Y = 30;
    public static double END_HEADING = 0;

    @Override
    public void runOpMode() {
        WingsBot2025 robot = new WingsBot2025(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if (opModeIsActive()) {

            // Starting Pose
            Pose2d startPose = new Pose2d(START_POSE_X,START_POSE_Y, Math.toRadians(START_HEADING));

            robot.localizer.setPose(startPose);

            // Move to far launch position
            Action action = robot.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d( WAYPOINT_FIRE_X , WAYPOINT_FIRE_Y, Math.toRadians( WAYPOINT_FIRE_HEADING)), Math.toRadians( WAYPOINT_FIRE_HEADING))
                    .build();
            Actions.runBlocking(new SequentialAction(action));

            // Fire three preloaded
            robot.fireArtifacts(FIRE_TIME, FLYWHEEL_SPINUP_TIME, FLYWHEEL_POWER);

            // Move to GPP spike approach position
            action = robot.actionBuilder(robot.localizer.getPose())
                    .splineToLinearHeading(new Pose2d( END_POSE_X , END_POSE_Y, Math.toRadians( END_HEADING)), Math.toRadians( END_HEADING))
                    .build();
            Actions.runBlocking(new SequentialAction(action));

            
            // stop autonomous and wait for finish
            telemetry.addLine("Testing complete. Stopping autonomous.");
            telemetry.update();
        }
    }
}
